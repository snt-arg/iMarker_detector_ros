#!/usr/bin/env python3

"""
📝 'iMarker Detector (ROS-based)' Software
    SPDX-FileCopyrightText: (2025) University of Luxembourg
    © 2025 University of Luxembourg
    Developed by: Ali TOURANI et al. at SnT / ARG.

'iMarker Detector (ROS-based)' is licensed under the "SNT NON-COMMERCIAL" License.
You may not use this file except in compliance with the License.
"""

import os
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.readConfig import readConfig
from marker_detector.arucoDetector import arucoMarkerDetector
from iMarker_algorithms.process import singleFrameProcessing, sequentialFrameProcessing


def main():
    # Initializing a ROS node
    rospy.init_node('iMarker_detector_off', anonymous=True)

    # Loading configuration values
    config = readConfig()
    if config is None:
        exit()

    # Get the config values
    cfgMode = config['mode']
    cfgMarker = config['marker']
    cfgGeneral = config['sensor']['general']
    cfgOffline = config['sensor']['offline']['rosbag']

    # Inform the user
    setupVariant = "Sequential Subtraction" if cfgMode['sequentialSubtraction'] else "Masking"
    rospy.loginfo(
        f'Framework started! [Offline Rosbag Captured by Single Vision Setup - {setupVariant}]')

    # Setup publishers
    rate = rospy.Rate(10)  # Publishing rate in Hz
    pubRaw = rospy.Publisher('raw_img', Image, queue_size=10)
    pubMask = rospy.Publisher('mask_img', Image, queue_size=10)
    pubMarker = rospy.Publisher('marker_img', Image, queue_size=10)
    pubMaskApplied = rospy.Publisher('mask_applied_img', Image, queue_size=10)

    # ROS Bridge
    bridge = CvBridge()

    # Variables
    prevFrame = None
    frameMask = None
    frameMaskApplied = None

    def fetchRawImage(msg):
        nonlocal prevFrame, frameMask, frameMaskApplied

        # Convert the ROS image message to an OpenCV image
        currFrame = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Change brightness
        currFrame = cv.convertScaleAbs(
            currFrame, alpha=cfgGeneral['brightness']['alpha'], beta=cfgGeneral['brightness']['beta'])

        # Only the first time, copy the current frame to the previous frame
        if prevFrame is None:
            prevFrame = np.copy(currFrame)

        # Process frames
        if cfgMode['sequentialSubtraction']:
            pFrame, cFrame, frameMask = sequentialFrameProcessing(
                prevFrame, currFrame, True, config)
            # Apply the mask
            frameMaskApplied = cv.bitwise_and(
                cFrame, cFrame, mask=frameMask)
        else:
            # Keep the original frame
            cFrameRGB = np.copy(currFrame)
            # Process the frames
            cFrame, frameMask = singleFrameProcessing(
                currFrame, True, config)
            # Apply the mask
            frameMaskApplied = cv.bitwise_and(
                cFrame, cFrame, mask=frameMask)

        # Convert the mask to RGB
        frameMask = cv.cvtColor(frameMask, cv.COLOR_GRAY2BGR)
        frameMaskApplied = cv.cvtColor(frameMaskApplied, cv.COLOR_BGR2RGB)

        # Preparing the frames for publishing
        frameRawRos = bridge.cv2_to_imgmsg(currFrame, "bgr8")
        frameMaskRos = bridge.cv2_to_imgmsg(frameMask, "bgr8")
        frameMaskAppliedRos = bridge.cv2_to_imgmsg(frameMaskApplied, "bgr8")

        # ArUco marker detection
        frameMarker = arucoMarkerDetector(
            frameMask, cfgMarker['detection']['dictionary'])
        frameMarkerRos = bridge.cv2_to_imgmsg(frameMarker, "bgr8")

        # Publishing the frames
        pubRaw.publish(frameRawRos)
        pubMask.publish(frameMaskRos)
        pubMarker.publish(frameMarkerRos)
        pubMaskApplied.publish(frameMaskAppliedRos)

        # Save the previous frame
        prevFrame = np.copy(currFrame)

    # Subscribe to the image topic from the rosbag
    rospy.Subscriber(cfgOffline['raw_image_topic'], Image, fetchRawImage)

    # Keep the node running
    rospy.spin()

    rospy.loginfo(
        f'Framework stopped! [Offline Rosbag Captured by Single Vision Setup - {setupVariant}]')
    rospy.signal_shutdown('Finished')


# Run the main function
if __name__ == '__main__':
    main()
