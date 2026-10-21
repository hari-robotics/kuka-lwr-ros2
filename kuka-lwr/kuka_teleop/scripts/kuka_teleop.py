#!/usr/bin/env python3

import rclpy, math, time, copy, threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from lwr_controllers.msg import PoseRPY
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Joy
# ROS 2: tf.transformations (imported but unused in ROS 1) has no Humble equivalent

master_value = PoseRPY()
slave_value = PoseRPY()
mode = "init_slave" 
#init_master to get initial master value, init_slave to get initial slave value, cmd to move the slave w/ the Sigma
node = None
pub = None # ROS 2: created after the node in LineTrajectoryCommand() ('/lwr/hierarchical_controller/command', depth 1)

k_pose = 0.3 #0.1
k_orientation = 0.5 # Not used now 

teleop = False

Xsd = PoseRPY()

theta = math.radians(-45) # -45 for simulation, 135 for real robot
R = [[math.cos(theta), math.sin(theta), 0],[-math.sin(theta), math.cos(theta), 0],[0, 0, 1]]

def Rotate(X, R):
	Xt = PoseRPY()
	Xt.position.x = R[0][0]*X.position.x + R[0][1]*X.position.y + R[0][2]*X.position.z
	Xt.position.y = R[1][0]*X.position.x + R[1][1]*X.position.y + R[1][2]*X.position.z
	Xt.position.z = R[2][0]*X.position.x + R[2][1]*X.position.y + R[2][2]*X.position.z
	return Xt	

def masterCallback(data):
	global pub, master_value, mode, Xsd, R, slave_data
	# ROS 2: a PoseStamped field only accepts a geometry_msgs/Pose, so the rotated pose
	# (a PoseRPY, ROS 1: data.pose = Rotate(data.pose,R)) is kept in data_pose
	data_pose = Rotate(data.pose,R)
	#Teleoperation command
	if (mode =="cmd") and (teleop == True):

		#Target Slave pose
		#position
		node.get_logger().info("Pos error: [%f %f %f]" % (Xsd.position.x-slave_value.position.x,Xsd.position.y-slave_value.position.y,Xsd.position.z-slave_value.position.z))

		Xsd.position.x = Xsd.position.x + k_pose*(data_pose.position.x - master_value.position.x)
		Xsd.position.y = Xsd.position.y + k_pose*(data_pose.position.y - master_value.position.y)
		Xsd.position.z = Xsd.position.z + k_pose*(data_pose.position.z - master_value.position.z)

		#print "target value", Xsd

		pub.publish(Xsd)

	#Update Master pose
	master_value.position = data_pose.position
		
	#Case first pose
	if mode =="init_master":
		mode = "cmd"
		print("master initial pose", master_value)
		node.get_clock().sleep_for(Duration(seconds=0.1)) # ROS 1 rospy.sleep(0.1), ROS time
		Xsd = slave_value

def slaveCallback(data):
	global mode
	global slave_value
	if mode == "init_slave":
		slave_value = data
		print("slave initial pose", slave_value)
		mode = "init_master"
		node.get_clock().sleep_for(Duration(seconds=0.1)) # ROS 1 rospy.sleep(0.1), ROS time
	else:
		slave_value = data

def teleopCallback(data):

	global teleop, Xsd
	if data.buttons[1] == 1:
		teleop = True
		Xsd = slave_value
		node.get_logger().info("Teleoperation: ON")
	else :
		teleop = False
		node.get_logger().info("Teleoperation: OFF")

def LineTrajectoryCommand():
	global node, pub
	rclpy.init()
	node = rclpy.create_node('LineTrajectory')
	pub = node.create_publisher(PoseRPY, '/lwr/hierarchical_controller/command', 1)
	# ROS 2: rospy ran every subscriber in its own thread (a callback could rospy.sleep without
	# blocking the others); one callback group per subscriber on a multi-threaded executor does
	# the same. The executor runs from here on so that the ROS clock (/clock) is updated.
	executor = MultiThreadedExecutor()
	executor.add_node(node)
	spin_thread = threading.Thread(target=executor.spin, daemon=True)
	spin_thread.start()
	print(node.get_clock().now().nanoseconds / 1e9)
	while node.get_clock().now().nanoseconds / 1e9 == 0.0:
		time.sleep(0.1)
		print(node.get_clock().now().nanoseconds / 1e9)
	
	sub = node.create_subscription(PoseStamped, '/sigma/sigma0/pose', masterCallback, 10,
		callback_group=MutuallyExclusiveCallbackGroup())
	sub_slave = node.create_subscription(PoseRPY, '/cartesian_position/pose/tooltip', slaveCallback, 10,
		callback_group=MutuallyExclusiveCallbackGroup())
	sub_teleop = node.create_subscription(Joy, '/sigma/sigma0/buttons', teleopCallback, 10,
		callback_group=MutuallyExclusiveCallbackGroup())
	node.get_clock().sleep_for(Duration(seconds=0.1))
	try:
		spin_thread.join()
	finally:
		executor.shutdown()
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ =='__main__':
	try:

		LineTrajectoryCommand()
	except KeyboardInterrupt: pass
