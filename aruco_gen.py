'''
/***************************************************************
 *  Script Name: aruco_gen.py 
 *  Project Name: PrecisionLanding
 *  Author: Alex Smith
 *  Date of Last Revision: 01/12/2025
 *
 *  Description:
 *      This script generates and saves an ArUco marker image.
 *
 *  Usage Instructions:
 *      Run the script to generate an ArUco marker image file.
 *      Example:
 *          python3 aruco_gen.py <marker_id>
 *
 *  Inputs:
 *      <marker_id>: ID of the ArUco marker to generate (integer).
 *
 *  Outputs:
 *      Saves the generated ArUco marker image as "marker<marker_id>.png".
 *
 *  Dependencies:
 *      OpenCV, DepthAI (installed as part of the installation guide in README.md)
 *
 *  Notes:
 *      Needs to be run with a command line argument specifying the marker ID.
 *
 ***************************************************************/
'''

#!/usr/bin/env python3

import sys # for CLI args
import cv2 # computer vision library
import depthai as dai # depthai library

marker_size = 100 # size of marker image in pixels
save_folder = "markers" # folder to save the generated marker

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) #init pre defined dictionary to use
print(dictionary)
print(type(dictionary))

for i in range(6, 8):
    print("Generating marker ID:", i)
    markerImage = cv2.aruco.generateImageMarker(dictionary, i, marker_size) # generate marker image with id from CLI
    cv2.imshow("marker", markerImage)
    cv2.imwrite(f"{save_folder}/4x4marker"+str(i)+".png", markerImage) 
    cv2.waitKey(1000) # display each marker for 1000ms