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
import rospkg
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.readConfig import readConfig
import iMarker_sensors.sensors.usb_interface as usb
from iMarker_algorithms.process import stereoFrameProcessing
from marker_detector.arucoMarkerDetector import arucoMarkerDetector


def main():
    # Initializing a ROS node
    rospy.init_node('iMarker_detector_usb', anonymous=True)

    # Initialize rospkg to get the package path
    rospack = rospkg.RosPack()

    try:
        # Get the package path
        packagePath = rospack.get_path('imarker_detector_ros')
        notFoundImagePath = os.path.join(packagePath, 'src/notFound.png')
    except rospkg.common.ResourceNotFound as e:
        rospy.logerr(f"[Error] ROS Package not found: {e}")
        return

    # Loading configuration values
    config = readConfig()
    if config is None:
        exit()

    # Get the config values
    cfgMarker = config['marker']
    cfgUsbCam = config['sensor']['usbCam']
    cfgGeneral = config['sensor']['general']

    # Inform the user
    rospy.loginfo(f'Framework started! [Double Vision USB Cameras Setup]')

    # Setup publishers
    rate = rospy.Rate(10)  # Publishing rate in Hz
    pubMask = rospy.Publisher('mask_img', Image, queue_size=10)
    pubMarker = rospy.Publisher('marker_img', Image, queue_size=10)
    pubRawL = rospy.Publisher('raw_img_left', Image, queue_size=10)
    pubRawR = rospy.Publisher('raw_img_right', Image, queue_size=10)

    # ROS Bridge
    bridge = CvBridge()

    # Camera
    capL = usb.createCameraObject(cfgUsbCam['ports']['lCam'])
    capR = usb.createCameraObject(cfgUsbCam['ports']['rCam'])

    # Check if the boost is enabled
    if cfgGeneral['fpsBoost']:
        capL.set(cv.CAP_PROP_FPS, 30.0)
        capR.set(cv.CAP_PROP_FPS, 30.0)

    # Prepare a notFound image
    notFoundImage = cv.imread(notFoundImagePath, cv.IMREAD_COLOR)

    while not rospy.is_shutdown():
        # Retrieve frames
        retL, frameLRaw = usb.grabImage(capL)
        retR, frameRRaw = usb.grabImage(capR)

        # Check if both cameras are connected
        if not retL and not retR:
            rospy.logerr("[Error] no camera is connected! Exiting ...")
            break

        # Flip the right frame
        if (cfgUsbCam['flipImage']):
            frameRRaw = cv.flip(frameRRaw, 1)

        # Change brightness
        frameLRaw = cv.convertScaleAbs(
            frameLRaw, alpha=cfgGeneral['brightness']['alpha'], beta=cfgGeneral['brightness']['beta'])
        frameRRaw = cv.convertScaleAbs(
            frameRRaw, alpha=cfgGeneral['brightness']['alpha'], beta=cfgGeneral['brightness']['beta'])

        # Process frames
        frameL, frameR, frameMask = stereoFrameProcessing(
            frameLRaw, frameRRaw, retL, retR, config, True)

        # Convert to RGB
        frameMask = cv.cvtColor(frameMask, cv.COLOR_GRAY2BGR)

        # Preparing the frames
        frameLRaw = frameLRaw if retL else notFoundImage
        frameRRaw = frameRRaw if retR else notFoundImage
        frameMask = frameMask if (retR and retL) else notFoundImage
        frameLRos = bridge.cv2_to_imgmsg(frameLRaw, "bgr8")
        frameRRos = bridge.cv2_to_imgmsg(frameRRaw, "bgr8")
        frameMaskRos = bridge.cv2_to_imgmsg(frameMask, "bgr8")

        # ArUco marker detection
        frameMarker = arucoMarkerDetector(
            frameMask, cfgMarker['detection']['dictionary'])
        frameMarker = frameMarker if (retR and retL) else notFoundImage
        frameMarkerRos = bridge.cv2_to_imgmsg(frameMarker, "bgr8")

        # Publishing the frames
        pubRawL.publish(frameLRos)
        pubRawR.publish(frameRRos)
        pubMask.publish(frameMaskRos)
        pubMarker.publish(frameMarkerRos)

        # Continue publishing
        rate.sleep()

    # Stop the pipeline
    capL.release()
    capR.release()
    rospy.loginfo(f'Framework stopped! [Double Vision USB Cameras Setup]')


# Run the main function
if __name__ == '__main__':
    main()
