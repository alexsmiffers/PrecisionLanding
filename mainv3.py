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
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
    detector = cv.aruco.ArucoDetector(dictionary, detectorParams)

    print("\x1b[31m"+"Starting mavlink communication..."+"\033[0m")
    # MAVLink connection
    if settings.get('Sim')==True:
        m = mavutil.mavlink_connection('tcp:192.168.10.201:5762', baud=settings.get('BAUD'))
        # m = mavutil.mavlink_connection('udpin:192.168.10.201:14550', baud=settings.get('BAUD'))
        print(m.mav)
    else:
        m = mavutil.mavlink_connection(settings.get('SERIAL_DEV'), baud=settings.get('BAUD'))
    # Wait for heartbeat so target system/component IDs are known
    try:
        m.wait_heartbeat(timeout=5)
    except Exception:
        pass  # continue anyway
    print(f"\x1b[31mHeartbeat received from system: {m.target_system} and component: {m.target_component}\033[0m")

    # Declare drone initial states
    if settings.get('Sim') == True: # only in SITL to prevent unintended behaviour on test drone
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0) # set to guided disarmed mode
        msg = m.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0) #arm drone
        msg = m.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, settings.get("ALTITUDE")) # takeoff to desired ALT
        msg = m.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

        m.mav.param_set_send(m.target_system, m.target_component, b'PLND_ENABLED', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # enable precision landing as the landing type
        m.mav.param_set_send(m.target_system, m.target_component, b'PLND_TYPE', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # set PLND input as mavlink
        m.mav.param_set_send(m.target_system, m.target_component, b'PLND_USEGPS', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32) # set PLND input as mavlink
        m.mav.param_set_send(m.target_system, m.target_component, b'SIM_PLD_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        # m.mav.param_set_send(m.target_system, m.target_component, b'SIM_PLD_LAT', -35.3632, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        # m.mav.param_set_send(m.target_system, m.target_component, b'SIM_PLD_LON', 149.1652, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        m.mav.param_set_send(m.target_system, m.target_component, b'RNGFND1_TYPE', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        m.mav.param_set_send(m.target_system, m.target_component, b'RNGFND1_MIN_CM', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        m.mav.param_set_send(m.target_system, m.target_component, b'RNGFND1_MAX_CM', 4000, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        m.mav.param_set_send(m.target_system, m.target_component, b'RNGFND1_PIN', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        m.mav.param_set_send(m.target_system, m.target_component, b'RNGFND1_SCALING', 12.12, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    # Frequency Match Init
    rate_hz = 20.0
    period = 1.0 / rate_hz
    next_t = time.time()
    longestLoop = 0
    iterations = 0

    # Pose Estimation Init
    rvec = None
    tvec = None
    x_b = y_b = z_b = 0.0
    position_valid = 0

    # Create pipeline
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
            loopstart = time.time()
            videoIn = videoQueue.get()
            if not isinstance(videoIn, dai.ImgFrame):
                print("\x1b[31m"+"Error: Received unexpected type from videoQueue."+"\033[0m")
                continue
            frame = videoIn.getCvFrame()

            # Detect ArUco markers
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None and settings.get('TAG_ID') in ids.flatten(): # when the desired tag is detected.
                idx = np.where(ids.flatten() == settings.get('TAG_ID'))[0][0]
                marker_corners = corners[idx]

                # Full pose to get body-frame xyz from camera frame
                obj_pts = np.array([
                    [-settings.get('TAG_SIZE')/2,  settings.get('TAG_SIZE')/2, 0],
                    [ settings.get('TAG_SIZE')/2,  settings.get('TAG_SIZE')/2, 0],
                    [ settings.get('TAG_SIZE')/2, -settings.get('TAG_SIZE')/2, 0],
                    [-settings.get('TAG_SIZE')/2, -settings.get('TAG_SIZE')/2, 0],
                ], dtype=np.float32)

                img_pts = marker_corners.reshape(-1,2).astype(np.float32)

                retval, rvec, tvec = cv.solvePnP(obj_pts, img_pts, camMatrix, distCoeffs, flags=cv.SOLVEPNP_IPPE_SQUARE)

                if retval:
                    # Camera frame: x right, y down, z forward
                    # Convert to NED: x forward, y right, z down
                    x_b = float(tvec[2])
                    y_b = float(tvec[0])
                    z_b = float(tvec[1])
                    position_valid = 1
                    # Derive angles from tvec
                    angle_x = math.atan2(y_b, x_b)  # body: forward=x_b, right=y_b
                    angle_y = math.atan2(z_b, x_b)  # body: down=z_b, forward=x_b
                    angle_x = math.atan2(y_b, x_b)  # body: forward=x_b, right=y_b
                    angle_y = math.atan2(z_b, x_b)  # body: down=z_b, forward=x_b
                    distance = math.sqrt(x_b**2 + y_b**2 + z_b**2)

                # Throttle to target rate
                tnow = time.time()
                if tnow >= next_t:
                    next_t += period
                    if position_valid:
                        print(f"Position (m): x={x_b:.2f}, y={y_b:.2f}, z={z_b:.2f};\n Angles (rad): x={angle_x:.2f}, y={angle_y:.2f};\nDistance={distance:.2f}")

                    time_usec = int(time.time()*1e6)
                    if settings.get('Sim') == True:
                        # LANDING_TARGET send to SITL
                        m.mav.landing_target_send(
                            time_usec,        # time_usec
                            0,                      # target_num
                            mavutil.mavlink.MAV_FRAME_BODY_FRD,
                            float(angle_x), float(angle_y),
                            float(distance),                    # distance (set 0 if using rangefinder)
                            settings.get('TAG_SIZE'), settings.get('TAG_SIZE'),               # size_x, size_y
                            x=float(x_b), 
                            y=float(y_b), 
                            z=float(z_b),          # position in body frame (if available)
                            # q=[1.0, 0.0, 0.0, 0.0],   # orientation (commented as unused)
                            type=mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                            position_valid=1
                        )
                    else:
                        # LANDING_TARGET send to Pixhawk
                        m.mav.landing_target_send(
                            int(tnow * 1e6),        # time_usec
                            0,                      # target_num
                            mavutil.mavlink.MAV_FRAME_BODY_NED,
                            float(angle_x), float(angle_y),
                            float(distance),                    # distance (set 0 if using rangefinder)
                            settings.get('TAG_SIZE'), settings.get('TAG_SIZE'),               # size_x, size_y
                            x=float(x_b), 
                            y=float(y_b), 
                            z=float(z_b),          # position in body frame (if available)
                            q=[1.0, 0.0, 0.0, 0.0],   # orientation (unused here)
                            type=mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                            position_valid=1
                        )

                # print("checkpoint pose computed")
                if settings.get('TEST_MODE') == True:
                    drawn = frame.copy()
                    if ids is not None and ids.any():
                        cv.aruco.drawDetectedMarkers(drawn, corners, ids)
                    if rvec is not None and tvec is not None:
                        cv.drawFrameAxes(drawn, camMatrix, distCoeffs, rvec, tvec, 0.010)
                    cv.imshow("video", drawn)
                    if cv.waitKey(1) == ord('q'):
                        print("\x1b[31m"+"Program Ended by user"+"\033[0m")
                        break
            else:
                print("\x1b[31m"+"No Aruco in FOV"+"\033[0m")
                if settings.get('TEST_MODE') == True:
                    cv.imshow("video", frame)
                    if cv.waitKey(1) == ord('q'):
                        print("\x1b[31m"+"Program Ended by user"+"\033[0m")
                        break
            if iterations>3:
                loopTime = time.time()-loopstart
                if loopTime > longestLoop:
                    longestLoop = loopTime
                    print("\033[93m" + "New longest time taken for a loop is: ", longestLoop, " on loop: ", iterations, "\033[0m")  
            iterations+=1    

# main
# Load settings from YAML
with open('settings.yaml', 'r') as file:
    settings = yaml.safe_load(file)

# Load camera matrix and distortion coefficients
camMatrix = np.load('mtx.npy')
distCoeffs = np.load('dist.npy')

aruco(settings, camMatrix, distCoeffs)
