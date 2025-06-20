#!/usr/bin/env python3
import cv2
import numpy as np

def rvec_to_euler(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll  = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = 0
    # print(np.degrees(roll), np.degrees(pitch), np.degrees(yaw))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

def detect_and_estimate_pose(frame, camera_matrix, dist_coeffs, marker_size=0.127):
    """
    frame         : OpenCV BGR image.
    camera_matrix : Camera calibration matrix.
    dist_coeffs   : Distortion coefficients.
    marker_size   : Physical marker size in meters.
    """
    # Convert image to grayscale.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Use a try/except block to support both newer and older OpenCV versions.
    try:
        # Newer versions (e.g., OpenCV 4.7+)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
    except (AttributeError, TypeError):
        # Older versions fallback
        aruco_dict = cv2.arucoRodrigues.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        try:
            # Estimate pose on the first detected marker.
            result = my_estimatePoseSingleMarkers(corners[0], marker_size, camera_matrix, dist_coeffs)
            rvecs, tvecs, _ = result
            rvec = rvecs[0]
            tvec = tvecs[0]
            # Convert rotation vector to Euler angles; extract yaw.
            rvec = rvec_to_euler(rvec)

            #rvec = [roll, pitch, yaw]
            #tvec = [x. y, z]
            return rvec, tvec
        except Exception as e:
            print("Pose estimation error:", e)
            return None
    else:
        return None

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    # print(rvecs, tvecs, trash)
    #tvecs = [lateral, longitudinal, vertical]
    return rvecs, tvecs, trash