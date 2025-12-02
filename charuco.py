'''
/***************************************************************
 *  Script Name: charuco.py
 *  Project Name: PrecisionLanding
 *  Author: Alex Smith
 *  Date of Last Revision: 02/12/2025
 *
 *  Description:
 *      This script detects ChArUco board in a video stream and estimates the poses in an output video feed.
 *
 *  Usage Instructions:
 *      Run the script after calibrating the camera using camera_calibration.py.
 *      Example:
 *          python3 charuco.py 1
 *          python3 charuco.py 2
 *          python3 charuco.py 3
 *
 *  Inputs:
 *      Must have camera calibration files mtx.npy and dist.npy in the same directory as the script and 
 *      OAK-D camera from DEPTHAI connected.
 *
 *  Outputs:
 *      Displays video feed with detected ChArUco board and the axes drawn.
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

# Import necessary libraries
import cv2 as cv
import depthai as dai
import numpy as np
import os, time, glob, argparse

def createBoard(squaresX, squaresY, squareLength, markerLength):
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
    board = cv.aruco.CharucoBoard((squaresX, squaresY), squareLength, markerLength, dictionary)
    boardImage = board.generateImage((600, 500), 10, 1)
    cv.imwrite("BoardImage.jpg", boardImage)

def calibrateCamera(rows, cols):
    count = 0 # used to count number of pictures taken
    loop = 0 # used to track if a loop has occurred
    pcount = -1 # used to track number of pictures taken in previous loop
    save_folder = "calibration_images"
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points
    objp = np.zeros((rows*cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objpoints = []
    imgpoints = []

    #Loop to take pictures
    with dai.Pipeline() as pipeline: # Create pipeline
        # Define source and output
        cam = pipeline.create(dai.node.Camera).build()
        videoQueue = cam.requestOutput((640,480)).createOutputQueue()

        # Connect to device and start pipeline
        pipeline.start()
        while pipeline.isRunning():
            videoIn = videoQueue.get()
            assert isinstance(videoIn, dai.ImgFrame)
            frame = videoIn.getCvFrame()
            cv.imshow("Video Feed", frame)
            if loop == 0 or count != pcount:
                print("Press 'i' to take a picture for calibration. Taken pictures: ", count)
                pcount = count
            key = cv.waitKey(1) & 0xFF
            if key == ord('i'):
                cv.imwrite(f"{save_folder}/calib{count}.jpg", frame)
                count += 1
           
            if count >= 10:
                print("Minimum pictures taken for calibration, stopping pipeline...")
                pipeline.stop()
                cv.destroyAllWindows()
            loop = 1
    
    print("Pipeline stopped, starting calibration in 1 second...")
    time.sleep(1) # wait for 1 seconds before starting calibration

    # Get images for calibration from folder
    BASE_DIR = os.path.dirname(os.path.abspath(__file__)) # path where this script lives
    save_folder = os.path.join(BASE_DIR, "calibration_images") # path for calibration image folder
    images = glob.glob(f"{save_folder}/*.jpg") # get all JPG images

    # Read images and find chessboard corners
    print("Images found:", len(images))
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, corners = cv.findChessboardCorners(gray, (cols, rows), None)

        if ret:
            objpoints.append(objp)

            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            cv.drawChessboardCorners(img, (cols, rows), corners2, ret)

        cv.imshow('img', img)
        cv.waitKey(500)   # show EVERY image for 0.5 seconds

    cv.destroyAllWindows()

    # Now do calibration once on all collected points
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        None,
        None
    )

    print("Camera matrix:\n", mtx)
    print("Distortion:\n", dist)

    np.save('mtx.npy', mtx)
    np.save('dist.npy', dist)

def detectCharucoBoardWithCalibrationPose(squaresX, squaresY, squareLength, markerLength):
    camMatrix = np.load('mtx.npy')
    distCoeffs = np.load('dist.npy')
    detectorParams = cv.aruco.DetectorParameters() # uses default parameters at the moment
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
    axisLength = 0.5 * (min(squaresX, squaresY) * (squareLength))
    board = cv.aruco.CharucoBoard((squaresX, squaresY), squareLength, markerLength, dictionary)

    charucoParams = cv.aruco.CharucoParameters()
    charucoParams.tryRefineMarkers = True # if tryRefineMarkers, refineDetectedMarkers() will be used in detectBoard()
    charucoParams.cameraMatrix = camMatrix # cameraMatrix can be used in detectBoard()
    charucoParams.distCoeffs = distCoeffs # distCoeffs can be used in detectBoard()
    charucoDetector = cv.aruco.CharucoDetector(board, charucoParams, detectorParams)

    totalTime = 0
    totalIterations = 0

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
            tick = cv.getTickCount()

            # detect markers and charuco corners
            charucoCorners, charucoIds, markerCorners, markerIds = charucoDetector.detectBoard(frame)

            if markerIds is not None:
                # estimate charuco board pose
                validPose = False
                if np.sum(camMatrix) != 0 and np.sum(distCoeffs) != 0 and charucoIds.size() >= 4:
                    objPoints, imgPoints = board.matchImagePoints(charucoCorners, charucoIds)
                    validPose, rvec, tvec = cv.solvePnP(objPoints, imgPoints, camMatrix, distCoeffs)
                
                # draw results
                drawn = frame.copy()
                if markerIds.size() > 0:
                    cv.aruco.drawDetectedMarkers(drawn, markerCorners)
                if charucoIds.size() > 0:
                    cv.aruco.drawDetectedCornersCharuco(drawn, charucoCorners, charucoIds, (255, 0, 0))
                if validPose:
                    cv.drawFrameAxes(drawn, camMatrix, distCoeffs, rvec, tvec, axisLength)
                else:
                    validPose = False
                    #do nothing if no markers detected
                currentTime = (cv.getTickCount() - tick) / cv.getTickFrequency()
                totalTime += currentTime
                totalIterations += 1
                if totalIterations % 30 == 0:
                    print(f"Detection Time = {currentTime * 1000} ms (Mean = {1000 * totalTime / totalIterations} ms)")
    
            cv.imshow("drawn", frame)
            if cv.waitKey(1) == ord('q'):
                break

# main
parser = argparse.ArgumentParser("Code for charuco board creation and detection of charuco board with and without camera calibration.")
parser.add_argument("function", nargs='?', const=1, default=3, help="1 to create a charuco board;\n2 to perform camera calibration;\n3 to detect charuco board with camera calibration and Pose Estimation", type=int)
parser.add_argument("squaresX", nargs='?', const=1, default=7, help="Number of squares in X direction", type=int)
parser.add_argument("squaresY", nargs='?', const=1, default=9, help="Number of squares in Y direction", type=int)
parser.add_argument("squareLength", nargs='?', const=1, default=0.04, help="Length of a square", type=float)
parser.add_argument("markerLength", nargs='?', const=1, default=0.02, help="Length of a marker", type=float)
args = parser.parse_args()
if args.function == 1:
    print("Chosen functionality is: ", args.function, " : Create ChArUco board image")
    createBoard(args.squaresX, args.squaresY, args.squareLength, args.markerLength)
    if os.path.exists("BoardImage.jpg"):
        print(f"The ChArUco board image succesfully created as file: 'BoardImage.jpg'.")
    else:
        print(f"The file 'BoardImage.jpg' does not exist.\nLikely means the board was not created.\nSuggested to repeat the script")
if args.function == 3:
    print("Chosen functionality is: ", args.function, " : Detect ChArUco board with camera calibration and Pose Estimation")
    detectCharucoBoardWithCalibrationPose(args.squaresX, args.squaresY, args.squareLength, args.markerLength)
elif args.function == 2:
    print("Chosen functionality is: ", args.function, " : Calibrate the camera", )
    calibrateCamera(args.squaresY, args.squaresX)
    if os.path.exists("mtx.npy") and os.path.exists("dist.npy"):
        print(f"Camera calibration files 'mtx.npy' and 'dist.npy' successfully created.")
    else:
        print(f"Camera calibration files do not exist.\nLikely means the calibration was not successful.\nSuggested to repeat the script")
# end main