#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math
import cv2

from aruco_detection import aruco_location

curr=(0,0)
dir=0

def pos_cb(data):
    global curr,dir
    curr=(data.position.x,data.position.y)
    dir=data.position.z

def rot_cb(data):
    rotate_to_final_location((data.position.x,data.position.y))

def send_vel(pub_vel,left,right):
    msg = Twist()
    msg.linear.x = left
    msg.linear.y = right
    print(left,right)
    pub_vel.publish(msg)


def move_straight(final_point):
    lin_tolerance=15
    lin_vel=2.0
    P=1.2
    while not rospy.is_shutdown():
        distance = math.sqrt((final_point[1] - curr[1]) ** 2 + (final_point[0] - curr[0]) ** 2)
        target_dir = math.atan2((final_point[1] - curr[1]), (final_point[0] - curr[0]))
        comp=P*(target_dir-dir)
        print("distance ",distance)
        if distance < lin_tolerance:
            send_vel(pub_vel,0,0)
            break
        else:
            if(target_dir-dir>0):
                send_vel(pub_vel,lin_vel+comp,max(1.5,lin_vel-comp))
            else:
                send_vel(pub_vel,max(1.5,lin_vel-comp),lin_vel+comp)
            # send_vel(pub_vel,lin_vel,lin_vel)
            # rospy.sleep(0.8)
            # distance = math.sqrt((final_point[1] - curr[1]) ** 2 + (final_point[0] - curr[0]) ** 2)
            # if distance < lin_tolerance:
            #     send_vel(pub_vel,0,0)
            #     break
            # rotate_to_final_location(final_point)
    print("yayayay")
 


def rotate_to_final_location(final_point):
    rot_vel = 1.5
    rot_tolerance = 0.15
    final_point = (329, 249)
    stopper=0
    while not rospy.is_shutdown():
        target_dir = math.atan2((final_point[1] - curr[1]), (final_point[0] - curr[0]))
        print(target_dir - dir)
        if math.fabs(dir - target_dir) < rot_tolerance:
            send_vel(pub_vel,0,0)
            stopper+=1
            if stopper==40:
                break
        else:       
                    stopper=0
            # if target_dir * dir > 0:
            #     print(target_dir - dir,"first")
            #     send_vel(pub_vel,2.0,-2.0)
            # else:
            #     if dir < 0:
                    # print(target_dir - dir,"second")
                    if 0 <= target_dir - dir <= math.pi:
                        send_vel(pub_vel,rot_vel,-rot_vel)
                    else:
                        send_vel(pub_vel,-rot_vel,rot_vel)
                # else:
                #     print(target_dir - dir,"third")
                #     if 0 >= target_dir - dir >= -math.pi:
                #         send_vel(pub_vel,-rot_vel,rot_vel)
                #     else:
                #         send_vel(pub_vel,rot_vel,-rot_vel)
        rate.sleep()
    move_straight(final_point)


if __name__ == "__main__":
    rospy.init_node('vel', anonymous=True)
    rate = rospy.Rate(20)
    pub_vel = rospy.Publisher('omni_vel', Twist, queue_size=1)
    rospy.Subscriber('/pose',Pose,pos_cb,queue_size=1)
    rospy.Subscriber('/rotate',Pose,rot_cb,queue_size=1)
    rospy.spin()