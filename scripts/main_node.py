#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

import utils

# def image_callback(msg):
#     bridge = CvBridge()
#     cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#     # Process the RGB image as desired
#     cv2.imshow("RGB Image", cv_image)
#     cv2.waitKey(1)

def image_callback(msg):
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    depth_colormap = utils.process_depth(depth_image)
    # Process the RGB image as desired
    cv2.imshow("DEPTH image", depth_colormap)
    cv2.waitKey(1)
    
def image_subscriber():
    rospy.init_node('rs_juan_node', anonymous=True)
    # para poder escuchar aligned_depth debe ser lanzado con:
    # roslaunch realsense2_camera rs_camera.launch aligned_depth:=true
    depth_topic_name = '/camera/aligned_depth_to_color/image_raw'
    rospy.Subscriber(depth_topic_name, Image, image_callback)
    # rgb_topic_name='/camera/color/image_raw'
    # rospy.Subscriber(rgb_topic_name, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber()
        rospy.loginfo("Holaaaa")
    except rospy.ROSInterruptException:
        pass

# roslaunch realsense2_camera rs_camera.launch align_depth:=true depth_width:=640 depth_height:=480 depth_fps:=15 color_width:=640 color_height:=480 color_fps:=15