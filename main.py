#!/usr/bin/env python3
from pymavlink import mavutil
import cv2 as cv
import depthai as dai
import numpy as np
import os, time, glob, argparse, math, yaml, sys

# Function Declarations
def center_from_corners(corners):
    # corners: (4,1,2) or similar; flatten:
    pts = corners.reshape(-1, 2)
    c = pts.mean(axis=0)
    return float(c[0]), float(c[1])

# Load settings from YAML
with open('settings.yaml', 'r') as f:
    settings = yaml.full_load(f)

# Load camera matrix and distortion coefficients
camMatrix = np.load('mtx.npy')
distCoeffs = np.load('dist.npy')

def charuco(settings, camMatrix, distCoeffs):
    print("Starting Charuco mode...")
    # Charuco Initialization
    detectorParams = cv.aruco.DetectorParameters() # uses default parameters at the moment
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
    axisLength = 0.5 * (min(settings.get('SQUARES_X'), settings.get('SQUARES_Y')) * (settings.get('SQUARE_LENGTH')))
    board = cv.aruco.CharucoBoard((settings.get('SQUARES_X'), settings.get('SQUARES_Y')), settings.get('SQUARE_LENGTH'), settings.get('MARKER_LENGTH'), dictionary)
    charucoParams = cv.aruco.CharucoParameters()
    charucoParams.tryRefineMarkers = True # if tryRefineMarkers, refineDetectedMarkers() will be used in detectBoard()
    charucoParams.cameraMatrix = camMatrix # cameraMatrix can be used in detectBoard()
    charucoParams.distCoeffs = distCoeffs # distCoeffs can be used in detectBoard()
    charucoDetector = cv.aruco.CharucoDetector(board, charucoParams, detectorParams)

    # MAVLink connection

    # m = mavutil.mavlink_connection(settings.get('SERIAL_DEV'), baud=settings.get('BAUD'))
    # # Wait for heartbeat so target system/component IDs are known (optional but helpful)
    # try:
    #     m.wait_heartbeat(timeout=5)
    # except Exception:
    #     pass  # continue anyway

    fx, fy = camMatrix[0,0], camMatrix[1,1]
    cx, cy = camMatrix[0,2], camMatrix[1,2]

    rate_hz = 20.0
    period = 1.0 / rate_hz
    print(period)
    next_t = time.time() # returns time in seconds since epoch

    rvec = None
    tvec = None

    # init camera
    with dai.Pipeline() as pipeline:
        # Define source and output
        cam = pipeline.create(dai.node.Camera).build(sensorFps = 30.3)
        videoQueue = cam.requestOutput((settings.get('FRAME_W'), settings.get('FRAME_H'))).createOutputQueue()

        # Connect to device and start pipeline
        pipeline.start()
        while pipeline.isRunning():
            videoIn = videoQueue.get()
            assert isinstance(videoIn, dai.ImgFrame)
            frame = videoIn.getCvFrame()
            # print("checkpoint cam online")

            # Detect ChArUco boards
            charucoCorners, charucoIds, markerCorners, markerIds = charucoDetector.detectBoard(frame)
            print(charucoCorners)
            # when a board is detected
            if markerIds is not None and markerIds.any() and charucoIds is not None and charucoIds.any():
                # print("checkpoint charuco detected")
                x_b = y_b = z_b = 0.0
                position_valid = 0
                validPose = False
                if np.sum(camMatrix) != 0 and np.sum(distCoeffs) != 0 and len(charucoIds) >= 6:
                    objPoints, imgPoints = board.matchImagePoints(charucoCorners, charucoIds)
                    validPose, rvec, tvec = cv.solvePnP(objPoints, imgPoints, camMatrix, distCoeffs)
                if validPose:
                    # Camera frame: x right, y down, z forward
                    # Convert to BODY(NED): x forward, y right, z down
                    x_b = float(tvec[2])
                    y_b = float(tvec[0])
                    z_b = float(tvec[1])
                    position_valid = 1
                    # Re-derive angles from tvec for consistency
                    angle_x = math.atan2(y_b, x_b)  # body: forward=x_b, right=y_b
                    angle_y = math.atan2(z_b, x_b)  # body: down=z_b, forward=x_b

                # Throttle to target rate
                tnow = time.time()
                print(tnow, next_t)
                if tnow >= next_t:
                    next_t += period

                    if validPose:
                        print(f"Position (m): x={x_b:.2f}, y={y_b:.2f}, z={z_b:.2f}; Angles (rad): x={angle_x:.2f}, y={angle_y:.2f}")
                    # LANDING_TARGET send (MAVLink2 fields via keyword args are supported by pymavlink)
                    
                    # m.mav.landing_target_send(
                    #     int(tnow * 1e6),        # time_usec
                    #     0,                      # target_num
                    #     mavutil.mavlink.MAV_FRAME_BODY_NED,
                    #     float(angle_x), float(angle_y),
                    #     0.0,                    # distance (set 0 if using rangefinder)
                    #     settings.get('SQUARE_LENGTH'), settings.get('SQUARE_LENGTH'),               # size_x, size_y
                    #     x_b, y_b, z_b,          # position in body frame (if available)
                    #     [1.0, 0.0, 0.0, 0.0],   # orientation (unused here)
                    #     mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                    #     position_valid
                    # )
                        # Optional visualization for bench testing:

                # print("checkpoint pose computed")
                if settings.get('TEST_MODE') == True:
                    # print("draw outputs started")
                    drawn = frame.copy()
                    if markerIds is not None and markerIds.any():
                        cv.aruco.drawDetectedMarkers(drawn, markerCorners)
                    if charucoIds is not None and charucoIds.any():
                        cv.aruco.drawDetectedCornersCharuco(drawn, charucoCorners, charucoIds, (255, 0, 0))
                    if rvec is not None and tvec is not None:
                        cv.drawFrameAxes(drawn, camMatrix, distCoeffs, rvec, tvec, axisLength)
                    cv.imshow("video", drawn)
                    if cv.waitKey(1) == ord('q'):
                        break

            else:
                cv.imshow("video", frame)
                if cv.waitKey(1) == ord('q'):
                    break

def aruco(settings, camMatrix, distCoeffs):
    print("Starting Aruco mode...")
    detectorParams = cv.aruco.DetectorParameters() # uses default parameters at the moment
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
    detector = cv.aruco.ArucoDetector(dictionary, detectorParams)

    # # MAVLink connection
    # m = mavutil.mavlink_connection(SERIAL_DEV, baud=BAUD)
    # # Wait for heartbeat so target system/component IDs are known (optional but helpful)
    # try:
    #     m.wait_heartbeat(timeout=5)
    # except Exception:
    #     pass  # continue anyway

    fx, fy = camMatrix[0,0], camMatrix[1,1]
    cx, cy = camMatrix[0,2], camMatrix[1,2]

    rate_hz = 20.0
    period = 1.0 / rate_hz
    next_t = time.time()

    rvec = None
    tvec = None

    # Create pipeline
    with dai.Pipeline() as pipeline:
        # Define source and output
        cam = pipeline.create(dai.node.Camera).build()
        videoQueue = cam.requestOutput((640,480)).createOutputQueue()

        # Connect to device and start pipeline
        pipeline.start()
        while pipeline.isRunning():
            videoIn = videoQueue.get()
            assert isinstance(videoIn, dai.ImgFrame)
            frame = videoIn.getCvFrame()

            # Detect ArUco markers
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None and settings.get('TAG_ID') in ids.flatten():
                idx = np.where(ids.flatten() == settings.get('TAG_ID'))[0][0]
                marker_corners = corners[idx]

                # Compute angles relative to optical axis
                u, v = center_from_corners(marker_corners)
                # Small-angle exact form:
                angle_x = math.atan2((u - cx) / fx, 1.0)
                angle_y = math.atan2((v - cy) / fy, 1.0)

                # Optional: full pose to get body-frame xyz from camera frame
                x_b = y_b = z_b = 0.0
                position_valid = 0
                if settings.get('USE_FULL_POSE') == True:
                    obj_pts = np.array([
                        [-settings.get('TAG_SIZE')/2,  settings.get('TAG_SIZE')/2, 0],
                        [ settings.get('TAG_SIZE')/2,  settings.get('TAG_SIZE')/2, 0],
                        [ settings.get('TAG_SIZE')/2, -settings.get('TAG_SIZE')/2, 0],
                        [-settings.get('TAG_SIZE')/2, -settings.get('TAG_SIZE')/2, 0],
                    ], dtype=np.float32)

                    img_pts = marker_corners.reshape(-1,2).astype(np.float32)

                    okp, rvec, tvec = cv.solvePnP(obj_pts, img_pts, camMatrix, distCoeffs, flags=cv.SOLVEPNP_IPPE_SQUARE)

                    if okp:
                        # Camera frame: x right, y down, z forward
                        # Convert to BODY(NED): x forward, y right, z down
                        # Approximate mapping: x_b =  tvec[2]; y_b =  tvec[0]; z_b =  tvec[1]
                        x_b = float(tvec[2])
                        y_b = float(tvec[0])
                        z_b = float(tvec[1])
                        position_valid = 1
                        # Re-derive angles from tvec for consistency
                        angle_x = math.atan2(y_b, x_b)  # body: forward=x_b, right=y_b
                        angle_y = math.atan2(z_b, x_b)  # body: down=z_b, forward=x_b

                # Throttle to target rate
                tnow = time.time()
                if tnow >= next_t:
                    next_t += period
                    if position_valid:
                        print(f"Position (m): x={x_b:.2f}, y={y_b:.2f}, z={z_b:.2f}; Angles (rad): x={angle_x:.2f}, y={angle_y:.2f}")
                    # # LANDING_TARGET send (MAVLink2 fields via keyword args are supported by pymavlink)
                    # m.mav.landing_target_send(
                    #     int(tnow * 1e6),        # time_usec
                    #     0,                      # target_num
                    #     mavutil.mavlink.MAV_FRAME_BODY_NED,
                    #     float(angle_x), float(angle_y),
                    #     0.0,                    # distance (set 0 if using rangefinder)
                    #     TAG_SIZE_M, TAG_SIZE_M, # size_x, size_y
                    #     x_b, y_b, z_b,          # position in body frame (if available)
                    #     [1.0, 0.0, 0.0, 0.0],   # orientation (unused here)
                    #     mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                    #     position_valid
                    # )

                # print("checkpoint pose computed")
                if settings.get('TEST_MODE') == True:
                    # print("draw outputs started")
                    drawn = frame.copy()
                    if ids is not None and ids.any():
                        cv.aruco.drawDetectedMarkers(drawn, corners, ids)
                    if rvec is not None and tvec is not None:
                        cv.drawFrameAxes(drawn, camMatrix, distCoeffs, rvec, tvec, 0.010)
                    cv.imshow("video", drawn)
                    if cv.waitKey(1) == ord('q'):
                        break

            else:
                cv.imshow("video", frame)
                if cv.waitKey(1) == ord('q'):
                    break

# main
if settings.get('DETECT_MODE') == 'CHARUCO':
    charuco(settings, camMatrix, distCoeffs)
elif settings.get('DETECT_MODE') == 'ARUCO':
    aruco(settings, camMatrix, distCoeffs)