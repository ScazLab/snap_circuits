#!/usr/bin/env python
import os

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from board_perception.part_classifier import BOARD_DATA


if __name__ == "__main__":
    rospy.init_node('test_image')
    pub = rospy.Publisher('board_images', Image, queue_size=10)
    bridge = CvBridge()
    img_path = os.path.join(BOARD_DATA, 'board_1' + '.png')
    img = cv2.imread(img_path)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo("[Test image publisher] Publishing {}".format(img_path))
        pub.publish(bridge.cv2_to_imgmsg(img))
        rate.sleep()
