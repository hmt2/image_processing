#!/usr/bin/env python

import cv2
import rospy
import numpy as np

from matplotlib import pyplot as plt
from cv_bridge          import CvBridge, CvBridgeError
from sensor_msgs.msg    import Image, CompressedImage

fps_time    = 0
bridge      = CvBridge()


class OpencvNode:


    def __init__(self):

        ### ROS init
        rospy.init_node('opencv_filter_node', anonymous=True)

        self.publisher_rate = 20

        self.sub_image      = rospy.Subscriber('/cozmo_camera/image', Image, self.imageCallback)
        self.pub_image      = rospy.Publisher('/cozmo/image_filter', Image)


    def imageCallback(self, msg):

        try:
            image = bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            print ('CvBridgeError', e)
        else:
            
            blur = cv2.GaussianBlur(image,(3,3),0)
            blur = cv2.bilateralFilter(blur,5,75,75)
           
            #image = cv2.resize(image, 320, 240)
            image = cv2.cvtColor(blur, cv2.COLOR_RGB2GRAY)
            self.pub_image.publish(bridge.cv2_to_imgmsg(image, encoding = "mono8"))



if __name__ == '__main__':

    try:
        pe  = OpencvNode()
        r   = rospy.Rate(pe.publisher_rate) # 10hz

        while not rospy.is_shutdown():
            r.sleep()

    except rospy.ROSInterruptException:
        pass

