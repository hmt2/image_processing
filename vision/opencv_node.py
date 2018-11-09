#!/usr/bin/env python3

import cv2
import rospy

from cv_bridge          import CvBridge, CvBridgeError
from sensor_msgs.msg    import Image, CompressedImage

fps_time    = 0
bridge      = CvBridge()


class OpencvNode:
    
    
    def __init__(self):
        
        ### ROS init
        rospy.init_node('opencv_node', anonymous=True)

        self.publisher_rate = 20

        self.sub_image      = rospy.Subscriber('/webcam/image_raw', Image, self.imageCallback, queue_size=0)
        self.pub_image      = rospy.Publisher('/socoro/image_emotion', Image)

    
    def imageCallback(self, msg):

        try:
            image = bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            print ('CvBridgeError', e)
        else:
            #####
            image = cv2.resize(image, 640, 480)

            self.pub_image.publish(bridge.cv2_to_imgmsg(image, encoding = "rgb8"))



if __name__ == '__main__':

    try:
        pe  = OpencvNode()
        r   = rospy.Rate(pe.publisher_rate) # 10hz
        
        while not rospy.is_shutdown():
            r.sleep()

    except rospy.ROSInterruptException:
        pass
    