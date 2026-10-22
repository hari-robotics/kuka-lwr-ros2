#!/usr/bin/env python3
#read image from external camera/frame grabber with cv2, crop to obtain US image from TELEMED software and publish it with ROS (dimensions: 512x512)
#Giorgia 27/06/2024
#Junling 04/04/2025, check the opencv lib function

from __future__ import print_function

# ROS 1: roslib.load_manifest('frame_grabber') (not needed in ROS 2)
import sys
import threading
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ROSInterruptException
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def spin(node):
    # Ctrl-C / SIGTERM shut the context down under the spinning executor
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass

def mouse_callback(event,x,y,flags,param):
    if event == cv2.EVENT_MOUSEMOVE:
        pixel_value = param[y,x]
        print(x,y)

def main(args):
    # ROS 2: the node must exist before its publishers (ROS 1 created the publisher first)
    rclpy.init(args=args)
    node = rclpy.create_node('image_topic_node_cropped')
    #image_pub = node.create_publisher(Image, "/frame_grabber/image_topic", 30)
    image_cropped_pub = node.create_publisher(Image, "/frame_grabber/image_cropped_topic", 2)
    bridge = CvBridge()
    # rclpy Rate needs the node spun in another thread; the loop below stays as in ROS 1
    spin_thread = threading.Thread(target=spin, args=(node,), daemon=True)
    spin_thread.start()
    rate = node.create_rate(10)

    cv_image = cv2.VideoCapture(0)
    #Set resolution
    cv_image.set(cv2.CAP_PROP_FRAME_WIDTH,1280) # cv2.CAP_PROP_FRAME_WIDTH
    cv_image.set(cv2.CAP_PROP_FRAME_HEIGHT,720) # # cv2.CAP_PROP_FRAME_WIDTH

    try:
        while rclpy.ok():
            # Capture the video frame by frame 
            ret, frame = cv_image.read()
            if not ret:
                # No frame (camera missing/disconnected): frame is None and slicing it would crash
                node.get_logger().warn("Failed to read frame from camera", throttle_duration_sec=2.0)
                rate.sleep()
                continue
            #print("Image dimensions:" + str(frame.shape))

            #frame_cropped = frame[80:560,400:880] #to set
            frame_cropped = frame[95:545,410:860] #to set
            #print("Cropped image dimensions:" + str(frame.shape)) 

            frame_cropped = cv2.resize(frame_cropped, (480,480))
            #print("Resized image dimensions:" + str(frame.shape))

            # Display the resulting frame
            cv2.imshow('frame', frame_cropped)

            cv2.setMouseCallback('frame',mouse_callback,frame_cropped)
            cv2.waitKey(1)

            # Publish image
            try:
                #image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
                image_msg = bridge.cv2_to_imgmsg(frame_cropped, "bgr8")
                image_msg.header.stamp = node.get_clock().now().to_msg()  # cv2_to_imgmsg leaves this at 0 otherwise
                if not rclpy.ok():
                    break  # shut down while reading the frame: the publisher is no longer valid
                image_cropped_pub.publish(image_msg)
            except CvBridgeError as e:
                print(e)

            rate.sleep()
    except (KeyboardInterrupt, ROSInterruptException):
        pass
    finally:
        # ROS 2: stop the spin thread before the interpreter exits
        cv_image.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)

if __name__ == '__main__':
    main(sys.argv)
