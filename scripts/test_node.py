#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # Process the RGB image as desired
    cv2.imshow("RGB Image", cv_image)
    cv2.waitKey(1)

def image_subscriber():
    rospy.init_node('rs_juan_node', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber()
        rospy.loginfo("Holaaaa")
    except rospy.ROSInterruptException:
        pass