#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def talker():
    cap = cv.VideoCapture(0)
    pub1 = rospy.Publisher('Imagen_RGB', Image)
    pub2 = rospy.Publisher('Imagen_Grey', Image)
    rospy.init_node('Convert_image')
    rate = rospy.Rate(30) # 30hz
    
    while not rospy.is_shutdown():
         ret, frame = cap.read()
         frame = np.asarray(frame)
         image_message1 = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
         pub1.publish(image_message1)
         
         gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
         image_message2 = bridge.cv2_to_imgmsg(gray_frame, encoding="passthrough")
         pub2.publish(image_message2)
         
         rate.sleep()
         
if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass