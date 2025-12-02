from pymavlink import mavutil
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
mtx = np.load('mtx.npy')
dist = np.load('dist.npy')
MARKER_SIZES = {
    23: 0.053,   # 5.3 cm
    56: 0.139,   # 13.9 cm
}
save_folder = "calibration_images"
count = 0
'''
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
'''
# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build()
    videoQueue = cam.requestOutput((640,480)).createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():

        # Get frame from camera
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        frame = videoIn.getCvFrame()
        '''
        # Get altitude above ground level (AGL) from MAVLink
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        agl = msg.alt - msg.terrain_alt
        agl = agl / 1000.0
        print("Altitude AGL:", agl)
        '''
        # Detect markers and estimate pose
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
        print(markerIds)

        '''
        if agl > 10000:  # only look for marker  if above 10 meters
            
            rvecs, tvecs = estimate_pose_multi_markers(
                markerCorners, 
                [56],  # only look for marker ID 56 when above 10m
                mtx, 
                dist,
                MARKER_SIZES
            )
        else:  # look for marker 23 if below 10 meters
            rvecs, tvecs = estimate_pose_multi_markers(
                markerCorners, 
                [23],  # only look for marker ID 23 when below 10m
                mtx, 
                dist,
                MARKER_SIZES
            )
        '''
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # print(rvecs, tvecs)
            cv2.destroyAllWindows()
            break


