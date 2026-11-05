#!/usr/bin/env python3
import rclpy
import time
import threading
import numpy as np
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point
from lwr_controllers.msg import PoseRPYStamped
from lwr_controllers.msg import CartesianImpedancePoint
from lwr_controllers.msg import Stiffness
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Wrench

ee_position = None
ee_orientation = None

def get_ee_position(data: PoseRPYStamped):
    global ee_position, ee_orientation
    ee_position = data.pose.position
    ee_orientation = data.pose.orientation

def get_quaternion_from_euler(roll, pitch, yaw):

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]


def spin(node):
    # Ctrl-C / SIGTERM shut the context down under the spinning executor
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main():
   global ee_position
   global ee_orientation
   # --------------Initialization, subscribers and publisher, rate -------------------------------------------
   # Init the node
   # Only SIGINT: on SIGTERM rclpy would shut down the context and leave rate.sleep() blocked forever
   rclpy.init(signal_handler_options=SignalHandlerOptions.SIGINT)
   node = rclpy.create_node('Cartesian_trajectory')
   # Subscriber: reads the current pose of the robot
   sub1 = node.create_subscription(PoseRPYStamped, '/cartesian_position/pose/end_effector', get_ee_position, 10)


   # Publisher for the reference
   pub = node.create_publisher(CartesianImpedancePoint, '/lwr/cartesian_impedance_controller/command', 10)
   # ROS 2: callbacks and Rate need the node spun in another thread; the loop below stays as in ROS 1
   spin_thread = threading.Thread(target=spin, args=(node,), daemon=True)
   spin_thread.start()
   time.sleep(1)

   # Define the node execution frequency
   rate = node.create_rate(10)

   pose = Pose()
   stiffness = Stiffness()
   damping = Stiffness()
   wrench = Wrench()
   reference = CartesianImpedancePoint()

   # Standard values for k and d (rclpy float64 fields reject int, hence 50.0 instead of 50)
   #k = [800, 800, 800, 100, 100, 100]
   stiffness.x = 20.0 #100
   stiffness.y = 20.0
   stiffness.z = 20.0
   stiffness.rx = 20.0 # 100
   stiffness.ry = 20.0
   stiffness.rz = 20.0

   #d = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
   damping.x = 0.8
   damping.y = 0.8
   damping.z = 0.8
   damping.rx = 0.8
   damping.ry = 0.8
   damping.rz = 0.8

      # wrench
   wrench.force.x = 0.0
   wrench.force.y = 0.0
   wrench.force.z = 0.0
   wrench.torque.x = 0.0
   wrench.torque.y = 0.0
   wrench.torque.z = 0.0

   try:
    # Without a pose the first command would be invalid (ROS 1 crashed with NameError here)
    while rclpy.ok() and ee_position is None:
     node.get_logger().info('Waiting for /cartesian_position/pose/end_effector ...', throttle_duration_sec=2.0)
     rate.sleep()

    while rclpy.ok():
     # PoseRPY.position is a Vector3; rclpy requires a Point for Pose.position
     pose.position = Point(x=ee_position.x, y=ee_position.y, z=ee_position.z)
     # tf.transformations does not exist in ROS 2; same roll/pitch/yaw (sxyz) convention
     quaternion = get_quaternion_from_euler(ee_orientation.roll,ee_orientation.pitch, ee_orientation.yaw)
     pose.orientation.x = quaternion[0]
     pose.orientation.y = quaternion[1]
     pose.orientation.z = quaternion[2]
     pose.orientation.w = quaternion[3]
     reference.x_fri = pose
     reference.k_fri = stiffness
     reference.d_fri = damping
     reference.f_fri = wrench
     pub.publish(reference)
     rate.sleep()
   except KeyboardInterrupt:
    pass
   finally:
    node.destroy_node()
    if rclpy.ok():
     rclpy.shutdown()


if __name__ == '__main__':
    main()
