'''
/***************************************************************
 *  Script Name: camera_calibration.py
 *  Project Name: PrecisionLanding
 *  Author: Alex Smith
 *  Date of Last Revision: 27/04/2024
 *
 *  Description:
 *      This script performs camera calibration using a series of chessboard images.
 *      It calculates the camera matrix and distortion coefficients.
 *
 *  Usage Instructions:
 *      Run the script in a directory containing chessboard images (.jpg).
 *      Example:
 *          python3 camera_calibration.py
 *
 *  Inputs:
 *      Must have chessboard images from the camera in .jpg file format in the "calibration_images" folder.
 *
 *  Outputs:
 *      Camera matrix and distortion coefficients saved as mtx.npy and dist.npy files.
 *
 *  Dependencies:
 *      OpenCV, NumPy (installed as part of the installation guide in README.md)
 *
 *  Notes:
 *      Needs to be modified to have a more appropriate chessboard size for the camera being calibrated.
 *
 ***************************************************************/
'''

import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

objpoints = []
imgpoints = []

save_folder = "calibration_images" # make sure this folder exists and contains the chessboard images
images = glob.glob(f"{save_folder}/*.jpg")

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

    if ret:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        cv.drawChessboardCorners(img, (7, 6), corners2, ret)

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
