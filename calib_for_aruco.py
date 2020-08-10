#!/usr/bin/env python
import numpy as np
import cv2
import glob
import math
import os
import rospy
from multi_robot.msg import calib_msgs
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg


def cal():
    rospy.loginfo("===================================START CALIBRATION===================================")
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('/home/mun/catkin_ws/src/multi_robot/src/camera_cal_img/*.png')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (9, 6), corners, ret)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    rospy.loginfo("===================================END CALIBRATION===================================")
    return mtx, dist
    
def main():
    rospy.init_node("calib_for_aruco")
    mtx, dist = cal()
    calib_pub = rospy.Publisher('/calib_msg', calib_msgs, queue_size=10)
    calib = calib_msgs()

    calib.mtx0 = mtx[0]
    calib.mtx1 = mtx[1]
    calib.mtx2 = mtx[2]
    calib.dist = dist
    
    calib_pub.publish(calib)
    
if __name__ == "__main__":
    main()
    
    
    
   
