#! /usr/bin/env python3

#registeres all variables: time_sec represents the time of the command;

import threading
import rclpy
from rclpy.duration import Duration
import csv
from geometry_msgs.msg import Vector3, PointStamped, Point, Twist, TwistStamped
from std_msgs.msg import Float32, Bool
# from rospy.names import global_name  (ROS 1 only, unused)
from lwr_controllers.msg import PoseRPY, ArmState




def get_sensor_force(data):
    global sensor_force, sensor_torque
    sensor_force = data.linear
    sensor_torque = data.angular

def get_orientation(msg):
    global ee_euler
    ee_euler = msg.orientation 

# ROS 1: filename = rospy.get_param("/file_name") at import (global parameter, set with
# `rosparam set file_name 'probe_data2.csv'`). ROS 2 has no global parameters: it is the node
# parameter file_name, read in main() once the node exists:
#   ros2 run sriforcesensor record_calib.py --ros-args -p file_name:=probe_data2.csv
filename = None

def main():
    
    global sensor_force
    global sensor_torque
    global ee_euler
    global filename
    # ROS 2: no anonymous node names (ROS 1 anonymous=True appended a random suffix)
    rclpy.init()
    node = rclpy.create_node('data_recording')
    filename = node.declare_parameter('file_name', rclpy.Parameter.Type.STRING).value
    if filename is None:
        raise KeyError('file_name')  # as rospy.get_param() without default
    node.create_subscription(Twist, '/Force', get_sensor_force, 10)
    node.create_subscription(PoseRPY, '/cartesian_position/pose/end_effector', get_orientation, 10)
    # rclpy Rate needs the node spun in another thread; the loop below stays as in ROS 1
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    rate = node.create_rate(100)
    try:
        file = open(filename, 'w')
        #fieldnames = ['time_sec','time_nsec','sensor_force_filt_x', 'sensor_force_filt_y','sensor_force_filt_z','sensor_force_x', 'sensor_force_y', 'sensor_force_z']
        fieldnames = ['sensor_force_x', 'sensor_force_y', 'sensor_force_z','roll','pitch','yaw']

        writer = csv.DictWriter(file, fieldnames=fieldnames, dialect='excel')
        writer.writeheader()
        # ROS 1 rospy.sleep(1) follows ROS time (/clock under use_sim_time)
        node.get_clock().sleep_for(Duration(seconds=1))

        while rclpy.ok():

            #writer.writerow({'time_sec':time_filt.stamp.secs,'time_nsec':time_filt.stamp.nsecs,'sensor_force_filt_x': sensor_force_filt.x,'sensor_force_filt_y':sensor_force_filt.y ,'sensor_force_filt_z':sensor_force_filt.z , 'sensor_force_x': sensor_force.x, 'sensor_force_y':sensor_force.y , 'sensor_force_z': sensor_force.z})
            writer.writerow({'sensor_force_x': sensor_force.x, 'sensor_force_y':sensor_force.y , 'sensor_force_z': sensor_force.z,'roll':ee_euler.roll,'pitch':ee_euler.pitch,'yaw':ee_euler.yaw})
            rate.sleep()

        file.close()
    except KeyboardInterrupt:
        pass
    finally:
        # ROS 2: stop the spin thread before the interpreter exits
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)

if __name__ == '__main__':
	main()




