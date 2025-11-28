#!/usr/bin/env python3

import cv2
import depthai as dai

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
print(dictionary)
print(type(dictionary))
markerImage = cv2.aruco.generateImageMarker(dictionary, 23, 200)
cv2.imwrite("marker23.png", markerImage)