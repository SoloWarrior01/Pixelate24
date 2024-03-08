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

P_rot=1.2
I_rot=0
D_rot=0.1

def pos_cb(data):
    global curr,dir
    curr=(data.position.x,data.position.y)
    dir=data.position.z

def rot_cb(data):
    rotate_to_final_location((data.position.x,data.position.y))

def pid_rot_cb(data):
    global P_rot,I_rot,D_rot
    P_rot = data.linear.x
    I_rot = data.linear.y
    D_rot = data.linear.z

def send_vel(pub_vel,left,right):
    msg = Twist()
    msg.linear.x = left
    msg.linear.y = right
    print("velocities",left,right)
    pub_vel.publish(msg)


def move_straight_1(final_point):
    lin_tolerance=15
    rot_tolerance=0.15
    lin_vel=1.6
    P_lin=1.2
    I_lin=0
    D_lin=0.1
    P_rot=1.2
    I_rot=0
    D_rot=0.4
    prev_rot=0
    ITerm_rot = 0
    max_rot = 0.2
    IMAX_rot= 0.20
    IMIN_rot= -0.20
    rot_scale = 1
    prev_time = rospy.get_time()
    stopper=0

    while not rospy.is_shutdown():
        curr_time = rospy.get_time()
        distance = math.sqrt((final_point[1] - curr[1]) ** 2 + (final_point[0] - curr[0]) ** 2)
        print("distance ",distance)
        target_dir = math.atan2((final_point[1] - curr[1]), (final_point[0] - curr[0]))

        err_rot = target_dir-dir
        dt=curr_time - prev_time
        ITerm_rot = ITerm_rot + dt*err_rot

        if(I_rot*ITerm_rot > IMAX_rot):
                ITerm_rot = IMAX_rot/I_rot
        if(I_rot*ITerm_rot < IMIN_rot):
                ITerm_rot = IMIN_rot/I_rot

        comp_rot=(P_rot*err_rot + D_rot*(err_rot-prev_rot)/dt + I_rot*ITerm_rot)/rot_scale
        round(comp_rot,2)
        print("rot",err_rot,comp_rot)

        if comp_rot > max_rot:
                comp_rot = max_rot
        elif comp_rot < - max_rot:
                comp_rot = -max_rot
            

        if math.fabs(err_rot) < rot_tolerance:
            stopper+=1
            if stopper==5:
                send_vel(pub_vel,0,0)
                break
        else:
            stopper=0
            if comp_rot>0:
                send_vel(pub_vel,lin_vel+comp_rot,-(lin_vel+comp_rot))
            else:
                send_vel(pub_vel,-(lin_vel-comp_rot),(lin_vel-comp_rot))

        # if distance < lin_tolerance:
        #     send_vel(pub_vel,0,0)
        #     break
        # else:
        #     if(target_dir-dir>0):
        #         send_vel(pub_vel,lin_vel+comp,max(1.5,lin_vel-comp))
        #     else:
        #         send_vel(pub_vel,max(1.5,lin_vel-comp),lin_vel+comp)
                                                # send_vel(pub_vel,lin_vel,lin_vel)
                                                # rospy.sleep(0.8)
                                                # distance = math.sqrt((final_point[1] - curr[1]) ** 2 + (final_point[0] - curr[0]) ** 2)
                                                # if distance < lin_tolerance:
                                                #     send_vel(pub_vel,0,0)
                                                #     break
                                                # rotate_to_final_location(final_point)
        prev_rot = err_rot
        prev_time = curr_time
        rate.sleep()
    print("yayayay")
 
def move_straight_2(final_point):
    lin_tolerance=15
    rot_tolerance=0.08
    lin_vel=2.0
    P_lin=1.2
    I_lin=0
    D_lin=0.1
    P_rot=1.5
    I_rot=1
    D_rot=0.3
    prev_rot=0
    ITerm_rot = 0
    max_rot = 0.3
    IMAX_rot= 0.20
    IMIN_rot= -0.20
    rot_scale = 15
    prev_time = rospy.get_time()
    stopper=0

    while not rospy.is_shutdown():
        curr_time = rospy.get_time()
        distance = math.sqrt((final_point[1] - curr[1]) ** 2 + (final_point[0] - curr[0]) ** 2)
        print("distance ",distance)
        target_dir = math.atan2((final_point[1] - curr[1]), (final_point[0] - curr[0]))

        err_rot = target_dir-dir
        dt=curr_time - prev_time
        ITerm_rot = ITerm_rot + dt*err_rot

        if(I_rot*ITerm_rot > IMAX_rot):
                ITerm_rot = IMAX_rot/I_rot
        if(I_rot*ITerm_rot < IMIN_rot):
                ITerm_rot = IMIN_rot/I_rot

        comp_rot=(P_rot*err_rot + D_rot*(err_rot-prev_rot)/dt + I_rot*ITerm_rot)/rot_scale
        round(comp_rot,2)
        print("rot",err_rot,comp_rot)

        if comp_rot > max_rot:
                comp_rot = max_rot
        elif comp_rot < - max_rot:
                comp_rot = -max_rot
            

        if distance < lin_tolerance:
            send_vel(pub_vel,0,0)
            break
        else:
            if math.fabs(err_rot) < rot_tolerance:
                comp_rot=0
            send_vel(pub_vel,lin_vel+comp_rot,lin_vel-comp_rot)
        prev_rot = err_rot
        prev_time = curr_time
        rate.sleep()
    print("yayayay")


def rotate_to_final_location(final_point):
    rot_vel = 1.5
    rot_tolerance = 0.15
    final_point = (255,268)
    stopper=0
    # while not rospy.is_shutdown():
    while False:
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
    move_straight_1(final_point)
    move_straight_2(final_point)


if __name__ == "__main__":
    rospy.init_node('vel', anonymous=True)
    rate = rospy.Rate(20)
    pub_vel = rospy.Publisher('omni_vel', Twist, queue_size=1)
    rospy.Subscriber('/pose',Pose,pos_cb,queue_size=1)
    rospy.Subscriber('/rotate',Pose,rot_cb,queue_size=1)
    rospy.Subscriber('/pid_rot',Twist,pid_rot_cb,queue_size=1)
    rospy.spin()