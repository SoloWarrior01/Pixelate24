#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import math
import rospy
from geometry_msgs.msg import Pose

ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
arucoParams = aruco.DetectorParameters_create()

# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
# cap.set(cv2.CAP_PROP_FPS, 60)

def send_pose(center,dir):
    pos = Pose()
    pos.position.x=center[0]
    pos.position.y=center[1]
    pos.position.z=dir
    pos_pub.publish(pos)
    
def aruco_location(image):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=arucoParams)
    if (ids is not None) and (corners is not None):
        # variable_translation = corners
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)        

            #Centers
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
  
            #Aruco ID's
            cv2.putText(image, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            print("[INFO] ArUco marker ID: {}".format(markerID))
            
            cv2.imshow("image", image)
            cv2.waitKey(1)
        current_point = (
            (corners[0][0] + corners[2][0]) / 2, (corners[0][1] + corners[2][1]) / 2)
            
        
        dir = math.atan2((corners[0][1] - corners[3][1]),
                                   (corners[0][0] - corners[3][0]))
    # print(current_point)
    # print(dir)
        send_pose(current_point,dir)

if __name__ == "__main__":
    rospy.init_node('cam')
    pos_pub = rospy.Publisher('pose', Pose, queue_size=1)
    cap = cv2.VideoCapture(2)
    while(True):
        _, img = cap.read()
        aruco_location(img)
