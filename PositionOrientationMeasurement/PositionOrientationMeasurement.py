# -*- coding: utf-8 -*-
import cv2
import cv2.aruco as aruco
import numpy as np
from math import pi
import csv
import time
arucoMarkerLength = 0.038    # [m]
distance_camera2prismbase = 532    # [mm]

class AR():

    def __init__(self, videoPort, cameraMatrix, distortionCoefficients):
        self.cap = cv2.VideoCapture(1)
        self.cameraMatrix = cameraMatrix
        self.distortionCoefficients = distortionCoefficients
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    def find_ARMarker(self):
        self.ret, self.frame = self.cap.read()
        if len(self.frame.shape) == 3:
            self.Height, self.Width, self.channels = self.frame.shape[:3]
        else:
            self.Height, self.Width = self.frame.shape[:2]
            self.channels = 1
        self.halfHeight = int(self.Height / 2)
        self.halfWidth = int(self.Width / 2)
        self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(self.frame, self.dictionary)
        aruco.drawDetectedMarkers(self.frame, self.corners, self.ids, (0,255,0))

    def show(self):
        cv2.imshow("result", self.frame)

    def get_exist_Marker(self):
        return len(self.corners)

    def is_exist_marker(self, i):
        num = self.get_exist_Marker()
        if i >= num:
            return False
        else:
            return True

    def release(self):
        self.cap.release()

    def get_ARMarker_points(self, i):
        if self.is_exist_marker(i):
            return self.corners[i]

    def get_average_point_marker(self, i):
        if self.is_exist_marker(i):
            points = self.get_ARMarker_points(i)
            points_reshape = np.reshape(np.array(points), (4, -1))
            G = np.mean(points_reshape, axis = 0)
            cv2.circle(self.frame, (int(G[0]), int(G[1])), 10, (255, 255, 255), 5)
            return G[0], G[1]

    def get_ARMarker_pose(self, i):
        if self.is_exist_marker(i):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(self.corners[i], arucoMarkerLength, self.cameraMatrix, self.distortionCoefficients)
            self.frame = aruco.drawAxis(self.frame, self.cameraMatrix, self.distortionCoefficients, rvec, tvec, 0.1)
            return rvec, tvec

    def get_degrees(self, i):
        if self.is_exist_marker(i):
            rvec, tvec, = self.get_ARMarker_pose(i)
            (roll_angle, pitch_angle, yaw_angle) =  rvec[0][0][0]*180/pi, rvec[0][0][1]*180/pi, rvec[0][0][2]*180/pi
            if pitch_angle < 0:
                roll_angle, pitch_angle, yaw_angle = -roll_angle, -pitch_angle, -yaw_angle
            return roll_angle, pitch_angle, yaw_angle, 1000*tvec
        
    def get_degrees_from_O(self, i):
        if self.is_exist_marker(i):
            rvec, tvec, = self.get_ARMarker_pose(i)

            tvec = np.squeeze(tvec)
            rvec = np.squeeze(rvec)
            rvec_matrix = cv2.Rodrigues(rvec)
            rvec_matrix = rvec_matrix[0]
            transpose_tvec = tvec[np.newaxis, :].T
            proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
            euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]

            rvec_mx = np.array([euler_angle[0],euler_angle[1],euler_angle[2]])
            tvec_mx = np.array([[1000*tvec[0]],[1000*tvec[1]],[1000*tvec[2]]])

            R_T = np.matrix([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
            t_W2O = np.array([[0.0], [0.0], [distance_camera2prismbase]])

            tvec_from_O = R_T*(tvec_mx-t_W2O)
            rvec_from_O = R_T*(rvec_mx)

            return float(rvec_from_O[0]), float(rvec_from_O[1]), float(rvec_from_O[2]), float(tvec_from_O[0]), float(tvec_from_O[1]), float(tvec_from_O[2])

    def get_degrees_from_O_old(self, i):
        if self.is_exist_marker(i):
            rvec, tvec, = self.get_ARMarker_pose(i)
            rvec_mx = np.array([[rvec[0][0][0]],[rvec[0][0][1]],[rvec[0][0][2]]])
            tvec_mx = np.array([[1000*tvec[0][0][0]],[1000*tvec[0][0][1]],[1000*tvec[0][0][2]]])

            R_T = np.matrix([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
            t_W2O = np.array([[0.0], [0.0], [distance_camera2prismbase]])

            tvec_from_O = R_T*((tvec_mx)-t_W2O)
            rvec_from_O = np.matrix([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])*(rvec_mx)

            (roll_angle, pitch_angle, yaw_angle) =  rvec_from_O[2]*180/pi, (rvec_from_O[1])*180/pi, rvec_from_O[0]*180/pi
            return float(roll_angle), float(pitch_angle), float(yaw_angle), float(tvec_from_O[0]), float(tvec_from_O[1]), float(tvec_from_O[2])

    def get_pose_only(self, i):
        if self.is_exist_marker(i):
            rvec, tvec, = self.get_ARMarker_pose(i)
            marker_pose = [1000*tvec[0][0][0], 1000*tvec[0][0][1], 1000*tvec[0][0][2]]  # [mm]
            return marker_pose
            
if __name__ == '__main__':

    camera_matrix = np.matrix([[345.37396215, 0.0, 311.0938498], [0.0, 345.32308962, 256.71679766], [0.0, 0.0, 1.0]])
    distortion = np.array([-0.00679909, 0.01737972, 0.00191054, -0.00314157,  -0.0114317])
    myCap = AR(0, camera_matrix, distortion)
    time_mark = int(time.time())
    stop_index = 0
    while True:
        myCap.find_ARMarker()
        myCap.get_average_point_marker(0)
        print(myCap.get_degrees_from_O(0))
        with open('./data/' + 'marker_pose' + str(time_mark) + '.csv', 'a', newline="") as f:
            writer = csv.writer(f, lineterminator="\n")
            marker_pose = myCap.get_degrees_from_O(0)
            if marker_pose is not None:
                writer.writerow(marker_pose)
                stop_index += 1
        myCap.show()
        if stop_index >= 30:
            myCap.release()
            cv2.destroyAllWindows()
            break
        if cv2.waitKey(1) > 0:
            myCap.release()
            cv2.destroyAllWindows()
            break
        # time.sleep(1)    # for checking printout