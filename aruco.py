'''
/***************************************************************
 *  Script Name: aruco.py
 *  Project Name: PrecisionLanding
 *  Author: Alex Smith
 *  Date of Last Revision: 01/12/2025
 *
 *  Description:
 *      This script detects ArUco markers in a video stream and estimates their poses in an output video feed.
 *
 *  Usage Instructions:
 *      Run the script after calibrating the camera using camera_calibration.py.
 *      Example:
 *          python3 aruco.py
 *
 *  Inputs:
 *      Must have camera calibration files mtx.npy and dist.npy in the same directory as the script and 
 *      OAK-D camera from DEPTHAI connected.
 *
 *  Outputs:
 *      Displays video feed with detected ArUco markers and their axes drawn.
 *
 *  Dependencies:
 *      OpenCV, DepthAI, NumPy (installed as part of the installation guide in README.md)
 *
 *  Notes:
 *      Make sure to define the correct marker sizes in the MARKER_SIZES dictionary. 
 *      Script should be translated into C++ for better performance if used in a real-time application.
 *      Need to tinker with DetectorParameters for better detection under different conditions.
 *
 ***************************************************************/
'''

#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import os


def estimate_pose_multi_markers(corners, ids, camera_matrix, dist_coeffs, marker_sizes):
    """
    corners: list of 4x1x2 marker corners from ArUco
    ids: array of marker IDs (Nx1)
    camera_matrix: intrinsic camera matrix from runnning camera_calibration.py
    dist_coeffs: distortion coefficients from running camera_calibration.py
    marker_sizes: dict {id: size_in_meters}
    """

    rvecs = {}
    tvecs = {}

    if ids is None:
        return rvecs, tvecs

    for c, id_val in zip(corners, ids.flatten()):
        
        if id_val not in marker_sizes:
            print(f"ID {id_val} has no defined marker size — skipping.")
            continue

        marker_length = marker_sizes[id_val]
        half = marker_length / 2.0

        # 3D coordinates of marker corners
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        img_points = c.reshape(-1, 1, 2)

        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if ok:
            rvecs[id_val] = rvec
            tvecs[id_val] = tvec
            print(f"SolvePNP successful for marker {id_val}")

    return rvecs, tvecs

detectorParams = cv2.aruco.DetectorParameters() # uses default parameters at the moment
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
count = 0
mtx = np.load('mtx.npy')
dist = np.load('dist.npy')
MARKER_SIZES = {
    23: 0.053,   # 5.3 cm
    56: 0.139,   # 13.9 cm
}
save_folder = "calibration_images"

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=10)
    videoQueue = cam.requestOutput((1600,1300)).createOutputQueue()
    cam.initialControl.setSharpness(0)     # range: 0..4, default: 1
    cam.initialControl.setBrightness(0)
    cam.initialControl.setLumaDenoise(1)   # range: 0..4, default: 1
    cam.initialControl.setChromaDenoise(4) # range: 0..4, default: 1
    cam.initialControl.setAutoExposureLimit(20000) # Max 5ms, if in lower light increase.

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        frame = videoIn.getCvFrame()
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

        rvecs, tvecs = estimate_pose_multi_markers(
            markerCorners, 
            markerIds,
            mtx, 
            dist,
            MARKER_SIZES
        )
        # draw axis for each marker
        for marker_id in rvecs:
            cv2.drawFrameAxes(
                frame, 
                mtx, 
                dist, 
                rvecs[marker_id], 
                tvecs[marker_id], 
                0.05  # adjust axis length in meters
            )

        cv2.imshow("drawn", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('i'):
            cv2.imwrite(f"{save_folder}/calib{count}.jpg", frame)
            count += 1

        elif key == ord('q'):
            print(rvecs, tvecs)
            cv2.destroyAllWindows()
            break

    