#!/usr/bin/env python3
from pymavlink import mavutil
from concurrent.futures import ThreadPoolExecutor
import cv2 as cv
import depthai as dai
import numpy as np
import time, math, yaml

# Function Declarations
def center_from_corners(corners):
    pts = corners.reshape(-1, 2)
    c = pts.mean(axis=0)
    return float(c[0]), float(c[1])

def controller(settings, camMatrix, distCoeffs, position_valid, x_b, y_b, z_b, angle_x, angle_y, period, next_t):
    """
    Communicates with an attached Ardupilot device.

    Args:
        settings:
        camMatrix:
        distCoeffs:
        position_valid:
        x_b, y_b, z_b:
        angle_x, angle_y:
        period, next_t:

    Returns:
        frame (numpy.ndarray): The captured image frame from the camera.
    """
    print("\x1b[31m"+"Starting mavlink communication..."+"\033[0m")
    # MAVLink connection
    if settings.get('Sim')==True:
        m = mavutil.mavlink_connection('tcp:192.168.10.233:5760', baud=settings.get('BAUD'))
    else:
        m = mavutil.mavlink_connection(settings.get('SERIAL_DEV'), baud=settings.get('BAUD'))
    # Wait for heartbeat so target system/component IDs are known (optional but helpful)
    try:
        m.wait_heartbeat(timeout=5)
    except Exception:
        pass  # continue anyway

    print(f"\x1b[31mHeartbeat received from system: {m.target_system} and component: {m.target_component}\033[0m")

    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)

    while True:
        # Throttle to target rate
        tnow = time.time()
        if tnow >= next_t:
            next_t += period
            if position_valid: # send landing target
                print(f"Position (m): x={x_b:.2f}, y={y_b:.2f}, z={z_b:.2f}; Angles (rad): x={angle_x:.2f}, y={angle_y:.2f}")
                if settings.get('Sim')==True:
                    m.mav.landing_target_send(
                        int(tnow*1e6),
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        0,
                        float(angle_x),
                        float(angle_y),
                        0.0,              # zero since using a rangefinder
                        settings['TAG_SIZE'],
                        settings['TAG_SIZE']
                    )
                else:
                    m.mav.landing_target_send(
                        int(tnow * 1e6),        # time_usec
                        0,                      # target_num
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        float(angle_x), float(angle_y),
                        0.0,                    # distance (set 0 if using rangefinder)
                        settings.get('TAG_SIZE'), settings.get('TAG_SIZE'),               # size_x, size_y
                        x_b, y_b, z_b,          # position in body frame (if available)
                        [1.0, 0.0, 0.0, 0.0],   # orientation (unused here)
                        mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                        position_valid
                    )

def poseEstimator(settings, camMatrix, distCoeffs, frame):
    """
    Estimates the pose of an ArUco marker in the given frame.

    Args:
        settings (dict): Configuration settings including TAG_ID, TAG_SIZE, and USE_FULL_POSE.
        camMatrix (np.ndarray): Camera intrinsic matrix.
        distCoeffs (np.ndarray): Camera distortion coefficients.
        frame (np.ndarray): Image frame from the camera.

    Returns:
        position_valid: 
        x_b, y_b, z_b: 
        angle_x, angle_y: 
    """
    print("\x1b[34m"+"Starting Aruco detection..."+"\033[0m")
    detectorParams = cv.aruco.DetectorParameters() # uses default parameters at the moment
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
    detector = cv.aruco.ArucoDetector(dictionary, detectorParams)

    fx, fy = camMatrix[0,0], camMatrix[1,1]
    cx, cy = camMatrix[0,2], camMatrix[1,2]

    rvec = None
    tvec = None
    angle_x = 0.0
    angle_y = 0.0

    while True:
        if frame is not None:
            # Detect ArUco markers
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None and settings.get('TAG_ID') in ids.flatten():
                idx = np.where(ids.flatten() == settings.get('TAG_ID'))[0][0]
                marker_corners = corners[idx]

                # Full pose to get body-frame xyz from camera frame
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
                        # Convert to NED: x forward, y right, z down
                        x_b = float(tvec[2])
                        y_b = float(tvec[0])
                        z_b = float(tvec[1])
                        position_valid = 1
                        # Derive angles from tvec
                        angle_x = math.atan2(y_b, x_b)  # body: forward=x_b, right=y_b
                        angle_y = math.atan2(z_b, x_b)  # body: down=z_b, forward=x_b

            yield position_valid, x_b, y_b, z_b, angle_x, angle_y

def camera():
    """
    Starts a camera feed and continuously returns frames.

    Args:
        none

    Returns:
        frame (numpy.ndarray): The captured image frame from the camera.
    """
    print("\x1b[32m"+"Starting camera feed..."+"\033[0m")
    with dai.Pipeline() as pipeline:
        # Define source and output
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=30)
        videoQueue = cam.requestOutput((640,480)).createOutputQueue()
        cam.initialControl.setSharpness(0)     # range: 0..4, default: 1
        cam.initialControl.setBrightness(0)
        cam.initialControl.setLumaDenoise(1)   # range: 0..4, default: 1
        cam.initialControl.setChromaDenoise(4) # range: 0..4, default: 1
        cam.initialControl.setAutoExposureLimit(5000) # Max 5ms, if in lower light increase.

        # Connect to device and start pipeline
        pipeline.start()
        while pipeline.isRunning():
            videoIn = videoQueue.get()
            assert isinstance(videoIn, dai.ImgFrame)
            frame = videoIn.getCvFrame()
            yield frame

# ---------------------main----------------------
# Load settings from YAML
with open('settings.yaml', 'r') as file:
    settings = yaml.safe_load(file)

# Load camera matrix and distortion coefficients
camMatrix = np.load('mtx.npy')
distCoeffs = np.load('dist.npy')

# controller frequency configuration
rate_hz = 20.0
period = 1.0 / rate_hz
next_t = time.time()

# initial declarations
position_valid = 0
x_b, y_b, z_b = 0.0
angle_x, angle_y = 0.0
frame = None

# Create a thread pool with 3 workers
with ThreadPoolExecutor(max_workers=3) as executor:
    # Submit three tasks to run in parallel
    executor.submit(poseEstimator, settings, camMatrix, distCoeffs, frame) # serial prints in blue
    executor.submit(camera) # serial prints in green
    executor.submit(controller, settings, camMatrix, distCoeffs, position_valid, x_b, y_b, z_b, angle_x, angle_y, period, next_t) # serial prints in red

