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
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.readConfig import readConfig
from iMarker_sensors.sensors import ids_interface
from iMarker_algorithms.process import stereoFrameProcessing
from marker_detector.arucoDetector import arucoDetector
from iMarker_sensors.sensors.config.presets import homographyMatrixPreset_iDS


def main():
    # Initializing a ROS node
    rospy.init_node('iMarker_detector_dv_ids', anonymous=True)

    # Initialize rospkg to get the package path
    rospack = rospkg.RosPack()

    try:
        # Get the package path
        packagePath = rospack.get_path('imarker_detector_ros')
        notFoundImagePath = os.path.join(packagePath, 'src/notFound.png')
        sensorsConfigPath = os.path.join(
            packagePath, 'src/iMarker_sensors/sensors/config')
    except rospkg.common.ResourceNotFound as e:
        rospy.logerr(f"[Error] ROS Package not found: {e}")
        return

    # Loading configuration values
    config = readConfig()
    if config is None:
        exit()

    # Get the config values
    cfgMarker = config['marker']
    cfgIDSCam = config['sensor']['ids']
    cfgGeneral = config['sensor']['general']

    # Inform the user
    rospy.loginfo(f'Framework started! [Dual-Vision iDS Cameras Setup]')

    # Setup publishers
    rate = rospy.Rate(10)  # Publishing rate in Hz
    pubMask = rospy.Publisher('mask_img', Image, queue_size=10)
    pubMarker = rospy.Publisher('marker_img', Image, queue_size=10)
    pubRawL = rospy.Publisher('raw_img_left', Image, queue_size=10)
    pubRawR = rospy.Publisher('raw_img_right', Image, queue_size=10)

    # ROS Bridge
    bridge = CvBridge()

    # Fetch the cameras
    cap1 = ids_interface.idsCamera(0)
    cap2 = ids_interface.idsCamera(1)

    # Get the calibration configuration
    cap1.getCalibrationConfig(sensorsConfigPath, 'cam1')
    cap2.getCalibrationConfig(sensorsConfigPath, 'cam2')

    # Set the ROI
    width = cfgIDSCam['roi']['cap1']['width']
    height = cfgIDSCam['roi']['cap1']['height']
    cap1.setROI(cfgIDSCam['roi']['cap1']['x'], cfgIDSCam['roi']['cap1']
                ['y'], width, height)
    cap2.setROI(cfgIDSCam['roi']['cap2']['x'], cfgIDSCam['roi']['cap2']
                ['y'], width, height)

    # Synchronize the cameras
    cap1.syncAsMaster()
    cap2.syncAsSlave()

    # Capture the frames
    cap1.startAquisition()
    cap2.startAquisition()

    # Set the exposure time
    cap1.setExposureTime(cfgIDSCam['exposureTime'])
    cap2.setExposureTime(cfgIDSCam['exposureTime'])

    # Prepare a notFound image
    notFoundImage = cv.imread(notFoundImagePath, cv.IMREAD_COLOR)

    while not rospy.is_shutdown():
        # Retrieve frames
        frame1Raw = cap1.getFrame()
        frame2Raw = cap2.getFrame()

        retL = False if (not np.any(frame1Raw)) else True
        retR = False if (not np.any(frame2Raw)) else True

        # Check if both cameras are connected
        if not (retL and retR):
            print(
                "\n[Error] Seems like cameras are not connected or not working properly. Exiting ...")
            break

        # Flip the right frame
        frame2Raw = cv.flip(frame2Raw, 1)

        # Change brightness
        beta = cfgGeneral['brightness']['beta']
        alpha = cfgGeneral['brightness']['alpha']
        frame1Raw = cv.convertScaleAbs(frame1Raw, alpha=alpha, beta=beta)
        frame2Raw = cv.convertScaleAbs(frame2Raw, alpha=alpha, beta=beta)

        # Add the homography matrix to the config
        config['presetMat'] = homographyMatrixPreset_iDS

        # Process frames
        frame1, frame2, frameMask = stereoFrameProcessing(
            frame1Raw, frame2Raw, retL, retR, config, False)

        # Convert to RGB
        frameMask = cv.cvtColor(frameMask, cv.COLOR_GRAY2BGR)

        # Preparing the frames
        frame1Raw = frame1Raw if retL else notFoundImage
        frame2Raw = frame2Raw if retR else notFoundImage
        frameMask = frameMask if (retR and retL) else notFoundImage
        frame1Ros = bridge.cv2_to_imgmsg(frame1Raw, "bgr8")
        frame2Ros = bridge.cv2_to_imgmsg(frame2Raw, "bgr8")
        frameMaskRos = bridge.cv2_to_imgmsg(frameMask, "bgr8")

        # ArUco marker detection
        frameMarker = arucoDetector(
            frameMask, cfgMarker['detection']['dictionary'])
        frameMarker = frameMarker if (retR and retL) else notFoundImage
        frameMarkerRos = bridge.cv2_to_imgmsg(frameMarker, "bgr8")

        # Publishing the frames
        pubRawL.publish(frame1Ros)
        pubRawR.publish(frame2Ros)
        pubMask.publish(frameMaskRos)
        pubMarker.publish(frameMarkerRos)

        # Continue publishing
        rate.sleep()

    # Stop the cameras
    cap1.closeLibrary()
    cap2.closeLibrary()
    rospy.loginfo(f'Framework stopped! [Dual-Vision iDS Cameras Setup]')


# Run the main function
if __name__ == '__main__':
    try:
        import ids_peak
        import ids_peak_ipl
        main()
    except ImportError:
        print(
            '[Error] Please install the `ids-peak` and `ids-peak-ipl` packages to use the iDS camera runner.')
