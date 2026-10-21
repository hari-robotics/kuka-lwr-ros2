#!/usr/bin/env python3
"""ROS 2 stiffness demo for lwr_controllers/CartesianImpedanceController.

Holds the pose read from ~/cartesian_pose at startup (x_fri is an absolute base-frame
pose in this controller) with zero wrench, and steps through stiffness levels so the
compliance can be felt by pushing the tool by hand. Ends by restoring the controller's
start-up stiffness/damping.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from lwr_controllers.msg import CartesianImpedancePoint

NS = '/lwr/cartesian_impedance_controller'
# (name, [kx, ky, kz, krx, kry, krz], [d]*6, seconds)
STAGES = [
    ('default (start-up values)', [800, 800, 200, 100, 100, 100], [0.8] * 6, 8),
    ('stiff',                     [1500, 1500, 1500, 150, 150, 150], [0.7] * 6, 10),
    ('medium',                    [500, 500, 500, 50, 50, 50], [0.7] * 6, 10),
    ('soft',                      [200, 200, 200, 20, 20, 20], [0.7] * 6, 10),
    ('restore default',           [800, 800, 200, 100, 100, 100], [0.8] * 6, 3),
]


class StiffnessDemo(Node):
    def __init__(self):
        super().__init__('stiffness_demo')
        self.pose = None
        self.x0 = None
        self.create_subscription(PoseStamped, NS + '/cartesian_pose', self.on_pose, 10)
        self.pub = self.create_publisher(CartesianImpedancePoint, NS + '/command', 1)

    def on_pose(self, msg):
        self.pose = msg.pose

    def send(self, k, d):
        m = CartesianImpedancePoint()
        m.header.stamp = self.get_clock().now().to_msg()
        m.x_fri = self.x0
        m.k_fri.x, m.k_fri.y, m.k_fri.z, m.k_fri.rx, m.k_fri.ry, m.k_fri.rz = map(float, k)
        m.d_fri.x, m.d_fri.y, m.d_fri.z, m.d_fri.rx, m.d_fri.ry, m.d_fri.rz = map(float, d)
        self.pub.publish(m)

    def offset_mm(self):
        p, q = self.pose.position, self.x0.position
        return 1000.0 * math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2)


def main():
    rclpy.init()
    node = StiffnessDemo()
    while rclpy.ok() and node.pose is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.x0 = node.pose
    p = node.x0.position
    node.get_logger().info(f'holding pose x={p.x:.4f} y={p.y:.4f} z={p.z:.4f}')
    while node.pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    for name, k, d, secs in STAGES:
        node.send(k, d)
        node.get_logger().info(f'stage "{name}": K={k} D={d[0]} for {secs}s -- push the tool gently')
        t_end = node.get_clock().now().nanoseconds + int(secs * 1e9)
        max_off = 0.0
        while node.get_clock().now().nanoseconds < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            max_off = max(max_off, node.offset_mm())
        node.get_logger().info(f'stage "{name}": max deviation from hold pose {max_off:.1f} mm')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
