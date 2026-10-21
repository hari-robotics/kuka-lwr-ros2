#! /usr/bin/env python3
from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
from sklearn import datasets
from sklearn import metrics
from sklearn.preprocessing import MinMaxScaler
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
import threading
import rclpy
from rclpy.duration import Duration
from lwr_controllers.msg import PoseRPY
from geometry_msgs.msg import Twist, Wrench
import pickle

euler  = PoseRPY()
force_sensor = Twist()
force_calib = Wrench()

def get_euler(msg):
    global euler
    euler = msg.orientation
def get_force(msg):
    global force_sensor
    force_sensor = msg


def main():

    # ROS 2: no anonymous node names (ROS 1 anonymous=True appended a random suffix)
    rclpy.init()
    node = rclpy.create_node('tool_compensation')
    node.create_subscription(PoseRPY,'/cartesian_position/pose/end_effector',get_euler,10)
    node.create_subscription(Twist,'/Force',get_force,10)
    force_pub = node.create_publisher(Wrench,'/Force_calibrated',10)
    # rclpy Rate needs the node spun in another thread; the loop below stays as in ROS 1
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    rate = node.create_rate(100)

    try:
        model = pickle.load(open('realProbe.sav','rb'))
        print(model)
        # ROS 1 rospy.sleep(1) follows ROS time (/clock under use_sim_time)
        node.get_clock().sleep_for(Duration(seconds=1))

        while rclpy.ok():

            angle = [[euler.roll,euler.pitch,euler.yaw]]
            tool_grav = model.predict(angle)
            #print(tool_grav)

            force_calib.force.x = force_sensor.linear.x - tool_grav[0][0]
            force_calib.force.y = force_sensor.linear.y-tool_grav[0][1]
            force_calib.force.z = force_sensor.linear.z-tool_grav[0][2]
            force_calib.torque = force_sensor.angular
            print(force_calib.force)
            force_pub.publish(force_calib)
        
            rate.sleep()
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



