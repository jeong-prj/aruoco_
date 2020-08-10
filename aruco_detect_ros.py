#!/usr/bin/env python
import numpy as np
import cv2
import glob
import math
import os
import rospy
from multi_robot.msg import aruco_msgs
from multi_robot.msg import calib_msgs
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg

class Aruco():
    def __init__(self):
        self.aruco_pub = rospy.Publisher('aruco_msg', aruco_msgs, queue_size=10)
        self.aruco = aruco_msgs()
        
        self.mtx = []
        self.dist = Float32()
        
    def arucomark(self):
        rospy.loginfo("===================================START DETECT===================================")
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        param = cv2.aruco.DetectorParameters_create()
        param.adaptiveThreshConstant = 10
        os.system('sudo modprobe bcm2835-v4l2')
        cam = cv2.VideoCapture(0)

        if cam.isOpened():
            while True:
                _, frame = cam.read()
                gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                coners, ids, point = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=param)

                if np.all(ids != None):
                    # rvecs, tvecs, objpoint = cv2.aruco.estimatePoseSingleMarkers(coners, 0.05, mtx, dist)
                    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(coners, 0.05, mtx, dist)
                    rvecs_temp = np.squeeze(rvecs)
                    rotation_matrix = np.zeros((3,3))
                    cv2.Rodrigues(rvecs, rotation_matrix)
                    camera_matrix = rotation_matrix.T * tvecs * -1
                    camera_matrix = np.squeeze(camera_matrix)
                    # camera_vector = np.zeros((1, 3))
                    camera_vector, _ = cv2.Rodrigues(camera_matrix)
                    print(camera_vector)
                    # print(camera_vector)
                    # camera_mat = rvecs_mat * rvecs + tvecs
                    # camera_mat_t = rvecs.T
                    # camera_mat = camera_mat_t + tvecs
                    # print("camera:", camera_mat)
                    rvecs_msg = rvecs.tolist()
                    tvecs_msg = tvecs.tolist()
                    

                    for i in range(0, ids.size):
                        rvecs_msg_x = rvecs_msg[i][0][0]
                        rvecs_msg_y = rvecs_msg[i][0][1]
                        rvecs_msg_z = rvecs_msg[i][0][2]
                        tvecs_msg_x = tvecs_msg[i][0][0]
                        tvecs_msg_y = tvecs_msg[i][0][1]
                        tvecs_msg_z = tvecs_msg[i][0][2]
                        rotation_vector_x = camera_vector[0]
                        rotation_vector_y = camera_vector[1]
                        rotation_vector_z = camera_vector[2]
                        
        
                        self.aruco.r_x = rvecs_msg_x
                        self.aruco.r_y = rvecs_msg_y
                        self.aruco.r_z = rvecs_msg_z
                        self.aruco.t_x = tvecs_msg_x
                        self.aruco.t_y = tvecs_msg_y
                        self.aruco.t_z = tvecs_msg_z
                        self.aruco.x = rotation_vector_x
                        self.aruco.y = rotation_vector_y
                        self.aruco.z = rotation_vector_z
                        self.aruco.id = int(ids[i])
                        rospy.loginfo(self.aruco)
                        self.aruco_pub.publish(self.aruco)
                        
                        frame = cv2.aruco.drawAxis(frame, self.mtx, self.dist, rvecs[i], tvecs[i], 0.05)

                    frame = cv2.aruco.drawDetectedMarkers(frame, coners, ids)
                    frame = cv2.aruco.drawDetectedMarkers(frame, point, borderColor=(0, 255, 0))

                cv2.imshow("result", frame)
                k = cv2.waitKey(100)
                if k == ord('q'):
                    break
        cam.release()
        cv2.destroyAllWindows()

    def CalibCallback(self, data):
        self.mtx.push_back(data.mtx0)
        self.mtx.push_back(data.mtx1)
        self.mtx.push_back(data.mtx2)
        self.dist = data.dist

    def main(self):
        rospy.init_node("aruco_detect")
        rospy.Subscriber("/calib_msg", calib_msgs, self.CalibCallback)
        arucomark()

aruco_detect = Aruco()

if __name__ == "__main__":
    aruco_detect.main()
    
    
    
    
    
