#!/usr/bin/env python3

import cv2
import depthai as dai

detectorParams = cv2.aruco.DetectorParameters()
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build()
    videoQueue = cam.requestOutput((4000,3000)).createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        frame = videoIn.getCvFrame()
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        cv2.imshow("drawn", frame)


        if cv2.waitKey(1) == ord("q"):
            break