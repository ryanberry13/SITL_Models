"""
OpenCV object tracker
"""

import cv2
import math
import threading

import numpy as np

from pymavlink import mavutil
from ultralytics import YOLO

# References
# - https://gstreamer.freedesktop.org/documentation/tutorials/basic/gstreamer-tools.html?gi-language=c
# - https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html
# 
# -https://medium.com/@MrBam44/object-tracking-with-opencv-and-python-7db8b233fab6
#
#


def main():

    # gst_pipeline = "videotestsrc ! videoconvert ! appsink"

    # gst_pipeline = "playbin uri=file:///Users/rhys/CloudStation/Projects/ArduPilot/GSoC/2024/AsifKhan/camera_tracker_gstreamer.mov ! appsink"
    # cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    # Capture video from a file
    cap = cv2.VideoCapture("/Users/rhys/CloudStation/Projects/ArduPilot/GSoC/2024/AsifKhan/camera_tracker_gstreamer.mov")

    if not cap.isOpened():
        print("Cannot open camera. Exiting")
        return


    # Object detection
    object_detector = cv2.createBackgroundSubtractorMOG2(history=100, varThreshold=50)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # If frame is read correctly, ret is True
        if not ret:
            print("Empty frame. Exiting...")
            break
        
        # Frame size
        height, width, _ = frame.shape

        # Extract region of interest
        roi = frame[300: 1000, 300: 1000]

        # Object detection
        mask = object_detector.apply(roi)
        _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Debug contours
        for cnt in contours:
            # Calculate area and remove small elements
            area = cv2.contourArea(cnt)
            if area > 100:
                # Draw contour
                cv2.drawContours(roi, [cnt], -1, (0, 255, 0), 2)

                # Draw bounding rectangle
                x, y, w, h = cv2.boundingRect(cnt) 
                cv2.rectangle(roi, (x,y), (x + w, y + h), (0, 255, 0), 3)

        cv2.imshow("ROI", roi)
        # cv2.imshow("Frame", frame)
        # cv2.imshow("Mask", mask)

        # Operations on the frame come here
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("frame", gray)

        if cv2.waitKey(1) == ord("q"):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
