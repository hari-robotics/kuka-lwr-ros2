#!/usr/bin/env python3
"""ROS 2 <-> ZeroMQ bridge, so the controllers Python environment needs no ROS. It only forwards.

ROS -> ZMQ (PUB, bound on pub_address), frames [b"ee_pose", json]:
    /cartesian_position/pose/end_effector (lwr_controllers/PoseRPY)
    {"id": int,
     "position": {"x", "y", "z"},
     "orientation": {"roll", "pitch", "yaw"}}
    Forwarded from the ROS subscription callback, every message, no rate limit.

ZMQ -> ROS (SUB, bound on sub_address), frames [b"command", json]:
    /lwr/cartesian_impedance_controller/command (lwr_controllers/CartesianImpedancePoint)
    {"frame_id": str,
     "x_fri": {"position": {"x", "y", "z"}, "orientation": {"x", "y", "z", "w"}},
     "k_fri": {"x", "y", "z", "rx", "ry", "rz"},
     "d_fri": {"x", "y", "z", "rx", "ry", "rz"},
     "f_fri": {"force": {"x", "y", "z"}, "torque": {"x", "y", "z"}}}
    Every field is required (a missing one drops the command rather than defaulting to 0).
    Forwarded as is, at most command_max_rate Hz (default 100) on average: a command over
    that rate is held and published as soon as the rate allows, replaced by any newer command
    in the meantime, so the newest one always arrives. A sender streaming at exactly 100 Hz
    loses nothing to its timing jitter (up to COMMAND_BURST commands may go back to back).
    Commands are setpoints: the sender interpolates (see kuka_sdk.interpolation).
    header.stamp is set by the bridge when the command is published.

Keep this protocol in sync with controllers/src/kuka_sdk.
"""

import json
import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import zmq

from lwr_controllers.msg import CartesianImpedancePoint, PoseRPY

EE_POSE_TOPIC = b"ee_pose"
COMMAND_TOPIC = b"command"
COMMAND_BURST = 2.0  # commands that may go out back to back before command_max_rate applies


VECTOR3_FIELDS = ("x", "y", "z")
QUATERNION_FIELDS = ("x", "y", "z", "w")
STIFFNESS_FIELDS = ("x", "y", "z", "rx", "ry", "rz")
RPY_FIELDS = ("roll", "pitch", "yaw")


def get_fields(msg, fields):
    return {name: getattr(msg, name) for name in fields}


def set_fields(msg, data, fields):
    for name in fields:
        value = float(data[name])  # KeyError if the sender left a field out
        if not math.isfinite(value):
            raise ValueError(f"{name} is {value}")
        setattr(msg, name, value)


def pose_rpy_to_dict(msg):
    return {
        "id": msg.id,
        "position": get_fields(msg.position, VECTOR3_FIELDS),
        "orientation": get_fields(msg.orientation, RPY_FIELDS),
    }


def dict_to_command(data, msg):
    msg.header.frame_id = str(data["frame_id"])
    set_fields(msg.x_fri.position, data["x_fri"]["position"], VECTOR3_FIELDS)
    set_fields(msg.x_fri.orientation, data["x_fri"]["orientation"], QUATERNION_FIELDS)
    o = msg.x_fri.orientation
    if not math.sqrt(o.x**2 + o.y**2 + o.z**2 + o.w**2) > 1e-6:
        raise ValueError("orientation quaternion has zero norm")
    set_fields(msg.k_fri, data["k_fri"], STIFFNESS_FIELDS)
    set_fields(msg.d_fri, data["d_fri"], STIFFNESS_FIELDS)
    set_fields(msg.f_fri.force, data["f_fri"]["force"], VECTOR3_FIELDS)
    set_fields(msg.f_fri.torque, data["f_fri"]["torque"], VECTOR3_FIELDS)
    return msg


class ZmqBridge(Node):
    def __init__(self):
        super().__init__('zmq_bridge')
        pub_address = self.declare_parameter('pub_address', 'tcp://*:5555').value
        sub_address = self.declare_parameter('sub_address', 'tcp://*:5556').value
        pose_topic = self.declare_parameter('pose_topic', '/cartesian_position/pose/end_effector').value
        command_topic = self.declare_parameter('command_topic', '/lwr/cartesian_impedance_controller/command').value
        command_max_rate = float(self.declare_parameter('command_max_rate', 100.0).value)  # Hz
        if not command_max_rate > 0.0:
            raise ValueError("command_max_rate must be > 0")
        self.command_period = 1.0 / command_max_rate

        self.zmq_context = zmq.Context()
        self.zmq_pub = self.zmq_context.socket(zmq.PUB)
        self.zmq_pub.setsockopt(zmq.LINGER, 0)
        self.zmq_pub.bind(pub_address)
        self.zmq_sub = self.zmq_context.socket(zmq.SUB)
        self.zmq_sub.setsockopt(zmq.LINGER, 0)
        self.zmq_sub.bind(sub_address)
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, COMMAND_TOPIC)

        self.sub_pose = self.create_subscription(PoseRPY, pose_topic, self.callback_pose, 10)
        self.pub_command = self.create_publisher(CartesianImpedancePoint, command_topic, 10)

        # zmq sockets are not thread safe: zmq_pub is only used by the executor, zmq_sub only by this thread
        self.command_thread = threading.Thread(target=self.receive_commands, daemon=True)
        self.command_thread.start()

        self.get_logger().info(
            f"{pose_topic} -> zmq '{EE_POSE_TOPIC.decode()}' on {pub_address}; "
            f"zmq '{COMMAND_TOPIC.decode()}' on {sub_address} -> {command_topic} (max {command_max_rate} Hz)")

    def callback_pose(self, msg):
        self.zmq_pub.send_multipart([EE_POSE_TOPIC, json.dumps(pose_rpy_to_dict(msg)).encode()])

    def parse_command(self, payload):
        try:
            return dict_to_command(json.loads(payload), CartesianImpedancePoint())
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f"Dropping malformed command: {e!r}", throttle_duration_sec=1.0)
            return None

    def publish_command(self, msg):
        try:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_command.publish(msg)
        except Exception as e:
            if not rclpy.ok():
                raise  # Ctrl-C shut the ROS context down: let receive_commands end
            self.get_logger().error(f"Publishing command failed: {e!r}")

    def receive_commands(self):
        # Blocks in recv until a command arrives; zmq_context.term() in destroy_node wakes it with ContextTerminated.
        # Rate limit: a token bucket refilled at command_max_rate. Its capacity of COMMAND_BURST lets a sender
        # streaming at exactly that rate through despite sleep jitter, while the average rate stays capped.
        # Only while a command is held does it wait with a deadline, until the next token.
        held = None  # newest command that arrived while no token was left
        tokens, refilled_at = COMMAND_BURST, time.monotonic()

        def refill():
            nonlocal tokens, refilled_at
            now = time.monotonic()
            tokens = min(COMMAND_BURST, tokens + (now - refilled_at) / self.command_period)
            refilled_at = now

        try:
            while True:
                if held is not None:
                    refill()
                    wait = (1.0 - tokens) * self.command_period
                    if wait <= 0.0 or not self.zmq_sub.poll(max(1, math.ceil(wait * 1000))):
                        refill()
                        tokens -= 1.0
                        self.publish_command(held)
                        held = None
                        continue
                topic, payload = self.zmq_sub.recv_multipart()
                msg = self.parse_command(payload)
                if msg is None:
                    continue
                refill()
                if held is None and tokens >= 1.0:
                    tokens -= 1.0
                    self.publish_command(msg)
                else:
                    held = msg  # replaces an older held command: only the newest matters
        except zmq.ContextTerminated:
            pass
        except Exception:  # publish_command only lets errors through once ROS has shut down
            if rclpy.ok():
                raise
        finally:
            self.zmq_sub.close()  # term() waits for every socket to close, so this must always run

    def destroy_node(self):
        self.zmq_pub.close()
        self.zmq_context.term()  # wakes receive_commands, which closes zmq_sub; returns once it has
        self.command_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZmqBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
