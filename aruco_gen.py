'''
/***************************************************************
 *  Script Name: aruco_gen.py 
 *  Project Name: PrecisionLanding
 *  Author: Alex Smith
 *  Date of Last Revision: 27/04/2024
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

marker_size = 200 # size of marker image in pixels
marker_id = int(sys.argv[1]) # ID of the marker to generate

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250) #init pre defined dictionary to use
print(dictionary)
print(type(dictionary))
markerImage = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size) # generate marker image with id from CLI
cv2.imwrite("marker"+str(marker_id)+".png", markerImage) 