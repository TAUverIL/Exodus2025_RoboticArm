import cv2
import cv2.aruco as aruco
import numpy as np
import math

def get_euler_angles(rotation_vector):
    # Calculate rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    
    # Get euler angles
    euler_angles = cv2.RQDecomp3x3(rotation_matrix)[0]
    
    # Convert to degrees
    angles_degrees = [math.degrees(angle) for angle in euler_angles]
    return angles_degrees

def custom_estimate_pose_single_markers(corners, marker_length, camera_matrix, dist_coeffs):
    rvecs = []
    tvecs = []

    # 3D points in marker coordinate system
    objp = np.array([
        [-marker_length / 2,  marker_length / 2, 0],
        [ marker_length / 2,  marker_length / 2, 0],
        [ marker_length / 2, -marker_length / 2, 0],
        [-marker_length / 2, -marker_length / 2, 0]
    ], dtype=np.float32)

    for corner in corners:
        image_points = corner[0].astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(objp, image_points, camera_matrix, dist_coeffs)
        if success:
            rvecs.append(rvec)
            tvecs.append(tvec)

    return rvecs, tvecs


def aruco_detection(frame):

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Dummy camera matrix and distortion coefficients (replace with real calibration for accuracy)
    # These are typical placeholder values for 640x480 resolution cameras
    camera_matrix = np.array([[600, 0, 320],
                              [0, 600, 240],
                              [0,   0,   1]], dtype=np.float32)

    dist_coeffs = np.zeros((5, 1))  # assuming no distortion for testing

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs = custom_estimate_pose_single_markers(corners, 0.2, camera_matrix, dist_coeffs)


        for i in range(len(ids)):
            # Draw axis on each detected marker
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03)

            # Calculate Euler angles
            euler_angles = get_euler_angles(rvecs[i])
            
            x, y = int(corners[i][0][0][0]), int(corners[i][0][0][1])
            distance = np.linalg.norm(tvecs[i])  # in meters

            x, y = int(corners[i][0][0][0]), int(corners[i][0][0][1])
            cv2.putText(frame, f"ID: {ids[i][0]}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Display distance in cm on frame
            cv2.putText(frame, f"Dist: {distance*100:.1f} cm", (x, y + 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Display angles in bottom left
            height = frame.shape[0]
            cv2.putText(frame, f"Roll: {euler_angles[0]:.1f}°", (10, height - 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Pitch: {euler_angles[1]:.1f}°", (10, height - 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Yaw: {euler_angles[2]:.1f}°", (10, height - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return frame