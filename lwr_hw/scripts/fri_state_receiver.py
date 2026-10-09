#!/usr/bin/env python3
"""ROS 2 FRI measurement receiver. Never sends any packets to the robot."""
import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from fri_receive_only import decode


JOINT_NAMES = ['lwr_a1_joint', 'lwr_a2_joint', 'lwr_e1_joint', 'lwr_a3_joint',
               'lwr_a4_joint', 'lwr_a5_joint', 'lwr_a6_joint']


class FriStateReceiver(Node):
    def __init__(self):
        super().__init__('fri_state_receiver', namespace='lwr')
        bind_ip = self.declare_parameter('bind_ip', '192.168.10.1').value
        port = self.declare_parameter('port', 49938).value
        self.robot_ip = self.declare_parameter('robot_ip', '192.168.10.2').value
        self.stale_timeout = float(self.declare_parameter('stale_timeout_s', 1.0).value)
        if self.stale_timeout <= 0:
            raise ValueError('stale_timeout_s must be positive')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Do not share the port with a control driver.
            self.socket.bind((bind_ip, port))
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise
        self.joints = self.create_publisher(JointState, 'joint_states', qos_profile_sensor_data)
        self.status = self.create_publisher(String, 'fri_receive_status', 10)
        self.count = self.invalid = self.foreign = self.gaps = 0
        self.previous_sequence = None
        self.first_receive = self.last_receive = None
        self.latest = None
        self.last_count = 0
        self.last_display = time.monotonic()
        self.create_timer(0.002, self.receive)
        self.create_timer(1.0, self.display)
        self.get_logger().info(
            f'Receive only: {bind_ip}:{port}, robot={self.robot_ip}; '
            'no FRI replies or commands. Publishing /lwr/joint_states in radians. '
            'Display order: A1 A2 E1 A3 A4 A5 A6.')

    def receive(self):
        for _ in range(256):
            try:
                data, source = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            if source[0] != self.robot_ip:
                self.foreign += 1
                continue
            packet = decode(data)
            if packet is None or not all(math.isfinite(v) for v in
                    packet['joint_position_rad'] + packet['measured_joint_torque_nm']):
                self.invalid += 1
                continue
            now = time.monotonic()
            if self.first_receive is None:
                self.first_receive = now
            self.last_receive = now
            self.latest = packet
            if self.previous_sequence is not None:
                delta = (packet['sequence'] - self.previous_sequence) % 65536
                if 1 < delta < 32768:
                    self.gaps += delta - 1
            self.previous_sequence = packet['sequence']
            self.count += 1
            msg = JointState()
            # Host ROS receive time; robot timestamp is preserved in the status.
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = packet['joint_position_rad']
            msg.effort = packet['measured_joint_torque_nm']
            # FRI contains no measured velocity field; leave velocity empty.
            self.joints.publish(msg)

    def display(self):
        now = time.monotonic()
        hz = (self.count - self.last_count) / (now - self.last_display)
        self.last_count, self.last_display = self.count, now
        age = now - self.last_receive if self.last_receive is not None else None
        fresh = age is not None and age <= self.stale_timeout
        report = dict(receive_only=True, transmitted_fri_packets=0, fresh=fresh,
                      packet_age_s=age, received_packets=self.count, observed_hz=hz,
                      invalid_packets=self.invalid, foreign_packets=self.foreign,
                      sequence_gaps=self.gaps, latest_measurement=self.latest)
        self.status.publish(String(data=json.dumps(report)))
        if not fresh:
            self.get_logger().warning('No fresh FRI measurements; joint_states are not republished from old data.')
            return
        packet = self.latest
        angles = ' '.join(f'{v:9.4f}' for v in packet['joint_position_deg'])
        self.get_logger().info(
            f"{hz:.1f} Hz | {packet['state']} | quality={packet['quality']} | "
            f"error={packet['error_bitfield']} warning={packet['warning_bitfield']} | "
            f'angles (deg): {angles}')

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = FriStateReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
