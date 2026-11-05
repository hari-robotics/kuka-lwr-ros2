#!/usr/bin/env python3
#read image from external camera/frame grabber with cv2 and publish it in a ROS topic
#Giorgia 27/06/2024

from __future__ import print_function

# ROS 1: roslib.load_manifest('frame_grabber') (not needed in ROS 2)
import sys
import threading
import rclpy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def on_mouse(event, x, y, flags, clicked):
    # Pixel index of the click in the image, frame[row, col] = frame[y, x]
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked[:] = [x, y]
        print("Clicked pixel: frame[" + str(y) + ", " + str(x) + "]")

def main(args):
    # ROS 2: the node must exist before its publisher (ROS 1 created the publisher first)
    rclpy.init(args=args)
    node = rclpy.create_node('image_topic_node')
    image_pub = node.create_publisher(Image, "image_topic", 1)
    bridge = CvBridge()
    # rclpy Rate needs the node spun in another thread; the loop below stays as in ROS 1
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    rate = node.create_rate(30)

    cv_image = cv2.VideoCapture(0)
    #Set resolution
    cv_image.set(cv2.CAP_PROP_FRAME_WIDTH,1280) # cv2.cv.CV_CAP_PROP_FRAME_
    cv_image.set(cv2.CAP_PROP_FRAME_HEIGHT,800)

    # The window has to exist before its mouse callback is set
    clicked = []
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', on_mouse, clicked)

    try:
        while rclpy.ok():
            # Capture the video frame by frame 
            ret, frame = cv_image.read()
            # print("Image dimensions:" + str(frame.shape)) 
        
            # Display the resulting frame, with the last click drawn on a copy so that it is not published
            display = frame.copy()
            if clicked:
                x, y = clicked
                cv2.drawMarker(display, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
                cv2.putText(display, "frame[" + str(y) + ", " + str(x) + "]", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow('frame', display)
            cv2.waitKey(1)

            # Publish image
            try:
                image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                print(e)

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
    main(sys.argv)
