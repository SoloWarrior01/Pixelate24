#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
from std_msgs.msg import UInt8
import math
import rospy
from geometry_msgs.msg import Pose

finished=0    

def finished_cb(data):
    global finished
    finished=data.data

def path_follow(path):
    global finished
    for i in path:
        finished=0
        pose = Pose()
        pose.position.x=i[0]
        pose.position.y=i[1]
        while pose_pub.get_num_connections()==0 and not rospy.is_shutdown():
            pass
        pose_pub.publish(pose)
        while finished!=1 and not rospy.is_shutdown():
            pass
    

if __name__ == "__main__":
    rospy.init_node('move',anonymous=True)
    pose_pub = rospy.Publisher('rotate', Pose, queue_size=1,latch=True)
    rospy.Subscriber('/finished',UInt8,finished_cb,queue_size=1)
    # path1=[(300,223)]
    path2=[(255,266),(255,220),(300,223),(300,266)]
    path_follow(path2)
    rospy.spin()
