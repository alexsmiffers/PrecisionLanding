#!/usr/bin/env python3
from pymavlink import mavutil
import cv2 as cv
import depthai as dai
import numpy as np
import time, math, yaml

# Function Declarations
def center_from_corners(corners):
    # corners: (4,1,2) or similar; flatten:
    pts = corners.reshape(-1, 2)
    c = pts.mean(axis=0)
    return float(c[0]), float(c[1])

def aruco(settings, camMatrix, distCoeffs):
    print("\x1b[34m"+"Starting Aruco mode..."+"\033[0m")
    detectorParams = cv.aruco.DetectorParameters() # uses default parameters at the moment
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
    detector = cv.aruco.ArucoDetector(dictionary, detectorParams)

    # print("\x1b[31m"+"Starting mavlink communication..."+"\033[0m")
    # # MAVLink connection
    # if settings.get('Sim')==True:
    #     # m = mavutil.mavlink_connection('tcp:192.168.10.201:5762', baud=settings.get('BAUD'))
    #     m = mavutil.mavlink_connection('udpin:127.0.0.1:14550', baud=settings.get('BAUD'))
    #     print(m.mav)
    # else:
    #     m = mavutil.mavlink_connection(settings.get('SERIAL_DEV'), baud=settings.get('BAUD'))
    # # Wait for heartbeat so target system/component IDs are known
    # try:
    #     m.wait_heartbeat(timeout=5)
    # except Exception:
    #     pass  # continue anyway
    # print(f"\x1b[31mHeartbeat received from system: {m.target_system} and component: {m.target_component}\033[0m")

    # set to guided mode to confirm control.
    # m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)

    # Declare drone initial states
    #if settings.get('Sim') == True: # only in SITL to prevent unintended behaviour on test drone
        # m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0) # set to guided disarmed mode
        # msg = m.recv_match(type='COMMAND_ACK', blocking=True)
        # print(msg)

        # m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0) #arm drone
        # msg = m.recv_match(type='COMMAND_ACK', blocking=True)
        # print(msg)

        # m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, settings.get("ALTITUDE")) # takeoff to desired ALT
        # msg = m.recv_match(type='COMMAND_ACK', blocking=True)
        # print(msg)

    # m.mav.param_set_send(m.target_system, m.target_component, b'PLND_ENABLED', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # enable precision landing as the landing type
    # m.mav.param_set_send(m.target_system, m.target_component, b'PLND_TYPE', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # set PLND input as mavlink
    # m.mav.param_set_send(m.target_system, m.target_component, b'PLND_USEGPS', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # set PLND to not use gps data

    # Frequency Match Init
    rate_hz = 20.0
    period = 1.0 / rate_hz
    next_t = time.time()

    # Angle Estimation Init
    fx, fy = camMatrix[0,0], camMatrix[1,1]
    cx, cy = camMatrix[0,2], camMatrix[1,2]

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
            if not isinstance(videoIn, dai.ImgFrame):
                print("\x1b[31m"+"Error: Received unexpected type from videoQueue."+"\033[0m")
                continue
            frame = videoIn.getCvFrame()

            # undistorting the image frame
            h,  w = frame.shape[:2]
            newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(camMatrix, distCoeffs, (w,h), 1, (w,h))
            undistortedFrame = cv.undistort(frame, camMatrix, distCoeffs, None, newCameraMatrix)
            # crop the image frame
            x, y, w, h = roi
            undistortedFrame = undistortedFrame[y:y+h, x:x+w]

            # Detect ArUco markers
            corners, ids, _ = detector.detectMarkers(undistortedFrame)
            if ids is not None and settings.get('TAG_ID') in ids.flatten(): # when the desired tag is detected.
                idx = np.where(ids.flatten() == settings.get('TAG_ID'))[0][0]
                marker_corners = corners[idx]

                # --- Compute angles relative to optical axis ---
                u, v = center_from_corners(marker_corners)
                print(u, v, cx, cy)
                # Small-angle exact form:
                angle_x = math.atan2((u - cx) / fx, 1.0)
                angle_y = math.atan2((v - cy) / fy, 1.0)

                # Throttle to target rate
                tnow = time.time()
                if tnow >= next_t:
                    next_t += period

                    # time_usec = int(time.time()*1e6)
                    # # LANDING_TARGET send to Pixhawk
                    # m.mav.landing_target_send(
                    #     time_usec,        # time_usec
                    #     0,                      # target_num
                    #     mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    #     float(angle_x), float(angle_y),
                    #     0,                    # distance (set 0 if using rangefinder)
                    #     settings.get('TAG_SIZE'), settings.get('TAG_SIZE'),               # size_x, size_y
                    # )
                    # print(f"Sent LANDING_TARGET: angle_x={math.degrees(angle_x):.2f}°, angle_y={math.degrees(angle_y):.2f}°")

                if settings.get('TEST_MODE'):
                    drawn = undistortedFrame.copy()
                    if ids is not None and ids.any():
                        cv.aruco.drawDetectedMarkers(drawn, corners, ids)
                    cv.imshow("video", drawn)
                    if cv.waitKey(1) == ord('q'):
                        print("\x1b[31m"+"Program Ended by user"+"\033[0m")
                        break
            else:
                print("\x1b[31m"+"No Aruco in FOV"+"\033[0m")
                if settings.get('TEST_MODE'):
                    cv.imshow("video", undistortedFrame)
                    if cv.waitKey(1) == ord('q'):
                        print("\x1b[31m"+"Program Ended by user"+"\033[0m")
                        break

# main
# Load settings from YAML
with open('settings.yaml', 'r') as file:
    settings = yaml.safe_load(file)

# Load camera matrix and distortion coefficients
try:
    camMatrix = np.load('mtx.npy')
    distCoeffs = np.load('dist.npy')
except FileNotFoundError:
    print("\x1b[31m"+"Camera calibration files not found. Please run camera_calibration.py first."+"\033[0m")
    exit(1) # exits program with error code 1 (general error)

aruco(settings, camMatrix, distCoeffs)
