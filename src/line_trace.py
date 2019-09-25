#!/usr/bin/env python
## coding: UTF-8
'''
line trace 
2019.9.16
by Sasaki and Miyazawa
'''
import sys
import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from lower_camera.msg import Line
#from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class LineTrace(object):
    def __init__(self):
        self._line_pub = rospy.Publisher('line', Line, queue_size=1) 
        self._hough_pub = rospy.Publisher('line_image', Image, queue_size=1) 
        self._image_sub = rospy.Subscriber('image_rect_color', Image, self.callback) #kyaribure-shongo no node
        self._bridge = CvBridge() # ros to OpenCV
        self._line = Line()
    
    def get_line(self, cv_image, minLineLength, maxLineGap):
        

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        image, contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) #?
        
        max_length = 0
        max_length_contour = -1
        false = False
        #nagasa saidai rinkaku sagasu
        for i in range(len(contours)) :
            tmp_length = cv2.arcLength(contours[i], false)
            if(max_length < tmp_length):
                max_length = tmp_length
                max_length_contour = i

        if (max_length_contour != -1):
            rows,cols = cv_image.shape[:2]
	    [vx,vy,x,y] = cv2.fitLine(contours[max_length_contour], cv2.DIST_L2, 0,0.01, 0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            line_image = cv2.line(cv_image,(cols-1,righty),(0,lefty),(0,255,0),2)
        return (line_image,vx,vy,x,y)


    def callback(self, data): 
        r = rospy.Rate(1)
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e
        
        line_image,vx,vy,x,y = self.get_line(cv_image, 10000,10)

        try:
            self._hough_pub.publish(self._bridge.cv2_to_imgmsg(line_image,'bgr8')) #OpenCV to ROS
        except CvBridgeError, e:
            print e
        
        line = Line()
        line.vx = vx
        line.vy = vy
        line.px = x
        line.py = y
        #rospy.loginfo(line)
        self._line_pub.publish(line)
        

        r.sleep()

if __name__ == '__main__':
    rospy.init_node('line_trace')
    color = LineTrace()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
