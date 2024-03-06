#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist

def nothing(x):
    pass

rospy.init_node("trackbar_speed")
vel_pub = rospy.Publisher('pid_rot', Twist, queue_size=10)
vel = Twist()
# Create a black image, a window
img = np.zeros((300,512,3), np.uint8)
cv2.namedWindow('image')



# create trackbars for color change

cv2.createTrackbar('P_rot','image',0,30,nothing)
cv2.createTrackbar('I_rot','image',0,3,nothing)
cv2.createTrackbar('D_rot','image',0,30,nothing)
# cv2.createTrackbar('scale','image',0,200,nothing)



# create switch for ON/OFF functionality

switch = '0 : OFF \n1 : ON'

cv2.createTrackbar(switch, 'image',1,1,nothing)



while not rospy.is_shutdown():

    cv2.imshow('image',img)

    k = cv2.waitKey(1000//30) & 0xFF

    if k == 27:
        vel.linear.x=0
        vel.linear.y=0
        vel.linear.z=0
        vel_pub.publish(vel)
        break



    # get current positions of four trackbars

    p_rot = cv2.getTrackbarPos('P_rot','image')

    i_rot = cv2.getTrackbarPos('I_rot','image')

    d_rot = cv2.getTrackbarPos('D_rot','image')

    s = cv2.getTrackbarPos(switch,'image')



    if s == 0:

        img[:] = 0
        vel.linear.x=0
        vel.linear.y=0
        vel.linear.z=0
        

    else:
        vel.linear.x=p_rot/10
        vel.linear.y=i_rot
        vel.linear.z=d_rot/10

    vel_pub.publish(vel)



cv2.destroyAllWindows()