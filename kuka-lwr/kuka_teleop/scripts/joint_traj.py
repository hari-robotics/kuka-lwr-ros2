#!/usr/bin/env python3
import rclpy
import threading
import time
from rclpy.duration import Duration
from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
# ROS 2: tf (imported but unused in ROS 1) has no Humble equivalent
import numpy as np


trajectory = JointTrajectory()


def main(): 
	
	rclpy.init()
	node = rclpy.create_node('joint_traj')
	# ROS 2: the Humble joint_trajectory_controller subscribes to ~/joint_trajectory (ROS 1 ~/command)
	#joint_sub = rospy.Subscriber("/lwr/joint_trajectory_controller/state",JointTrajectory,trajCallback)
	joint_pub = node.create_publisher(JointTrajectory, "/lwr/joint_trajectory_controller/joint_trajectory", 10)
	# rospy.sleep() follows ROS time: spin the node in another thread for the clock
	spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
	spin_thread.start()
	trajectory = JointTrajectory()
	trajectory.joint_names = ['lwr_a1_joint','lwr_a2_joint','lwr_e1_joint','lwr_a3_joint','lwr_a4_joint','lwr_a5_joint','lwr_a6_joint']
	#trajectory.header.stamp = rospy.Time.now()
	trajectoryPoint = JointTrajectoryPoint()
	# ROS 2: float64[] fields only accept floats (ROS 1 [0, 0, 0.0, 0., 0.0, 0,0])
	trajectoryPoint.positions = [0.0, 0.0, 0.0, 0., 0.0, 0.0,0.0]
	trajectoryPoint.velocities = [0.0, 1.0, 0.0, -1.0, 0.0, 2.0,0.0]
	trajectoryPoint.time_from_start = DurationMsg(sec=2) #rospy.Duration(time)
	trajectory.points.append(trajectoryPoint)
	node.get_clock().sleep_for(Duration(seconds=2))

	try:
		while rclpy.ok():
			joint_pub.publish(trajectory)
			print(trajectory)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
		spin_thread.join(timeout=1.0)



if __name__ == '__main__':
	main()
