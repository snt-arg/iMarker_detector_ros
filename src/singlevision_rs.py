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
from utils.readConfig import readConfig
from sensor_msgs.msg import Image, CameraInfo
from iMarker_sensors.sensors import rs_interface
from marker_detector.arucoDetector import arucoMarkerDetector
from iMarker_algorithms.process import sequentialFrameProcessing, singleFrameProcessing


def main():
    # Initializing a ROS node
    rospy.init_node('iMarker_detector_rs', anonymous=True)

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
    cfgMode = config['mode']
    cfgMarker = config['marker']
    cfgRS = config['sensor']['realSense']
    cfgGeneral = config['sensor']['general']

    # Inform the user
    setupVariant = "Sequential Subtraction" if cfgMode['temporalSubtraction'] else "Masking"
    rospy.loginfo(
        f'Framework started! [RealSense Single Vision Setup - {setupVariant}]')

    # Setup publishers
    rate = rospy.Rate(10)  # Publishing rate in Hz
    pubRaw = rospy.Publisher('raw_img', Image, queue_size=10)
    pubMask = rospy.Publisher('mask_img', Image, queue_size=10)
    pubMarker = rospy.Publisher('marker_img', Image, queue_size=10)
    pubMaskApplied = rospy.Publisher('mask_applied_img', Image, queue_size=10)
    pubCameraParams = rospy.Publisher(
        'rs_cam_params', CameraInfo, queue_size=10)

    # ROS Bridge
    bridge = CvBridge()

    # Variables
    prevFrame = None
    frameMask = None
    frameMaskApplied = None

    # Create an object
    resolution = (cfgRS['resolution']['width'], cfgRS['resolution']['height'])
    rs = rs_interface.rsCamera(resolution, cfgRS['fps'])

    # Create a pipeline
    rs.createPipeline()

    # Start the pipeline
    isPipelineStarted = rs.startPipeline()

    # Prepare a notFound image
    notFoundImage = cv.imread(notFoundImagePath, cv.IMREAD_COLOR)

    while not rospy.is_shutdown():
        # Check if the frames are valid
        if not isPipelineStarted:
            break

        # Wait for the next frames from the camera
        frames = rs.grabFrames()
        ret = False if frames is None else True

        # Check if the frames are valid
        if not ret:
            break

        # Get the color frame
        currFrame, cameraMatrix, distCoeffs = rs.getColorFrame(frames)

        # Change brightness
        currFrame = cv.convertScaleAbs(
            currFrame, alpha=cfgGeneral['brightness']['alpha'], beta=cfgGeneral['brightness']['beta'])

        # Only the first time, copy the current frame to the previous frame
        if prevFrame is None:
            prevFrame = np.copy(currFrame)

        # Process frames
        if (cfgMode['temporalSubtraction']):
            # Process the frames
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

        # Camera Params
        camInfoMsgs = getCameraInfo(currFrame.shape, cameraMatrix, distCoeffs)

        # Convert to RGB
        frameMask = cv.cvtColor(frameMask, cv.COLOR_GRAY2BGR)
        frameMaskApplied = cv.cvtColor(frameMaskApplied, cv.COLOR_BGR2RGB)

        # Preparing the frames
        frameRaw = currFrame if ret else notFoundImage
        frameMask = frameMask if ret else notFoundImage
        frameMaskApplied = frameMaskApplied if ret else notFoundImage
        frameRawRos = bridge.cv2_to_imgmsg(frameRaw, "bgr8")
        frameMaskRos = bridge.cv2_to_imgmsg(frameMask, "bgr8")
        frameMaskApplied = bridge.cv2_to_imgmsg(frameMaskApplied, "bgr8")

        # ArUco marker detection
        frameMarker = arucoMarkerDetector(
            frameMask, cameraMatrix, distCoeffs, cfgMarker['detection']['dictionary'],
            cfgMarker['structure']['size'])
        frameMarker = frameMarker if ret else notFoundImage
        frameMarkerRos = bridge.cv2_to_imgmsg(frameMarker, "bgr8")

        # Publishing the frames
        pubRaw.publish(frameRawRos)
        pubMask.publish(frameMaskRos)
        pubMarker.publish(frameMarkerRos)
        pubCameraParams.publish(camInfoMsgs)
        pubMaskApplied.publish(frameMaskApplied)

        # Save the previous frame
        prevFrame = np.copy(currFrame)

        # Continue publishing
        rate.sleep()

    # Stop the pipeline and close the windows
    if isPipelineStarted:
        rs.stopPipeline()
    rospy.loginfo(
        f'Framework stopped! [RealSense Single Vision Setup - {setupVariant}]')


def getCameraInfo(frameShape, cameraMatrix, distCoeffs):
    # Create a message and fill it with calibration params
    camInfoMsgs = CameraInfo()
    camInfoMsgs.header.frame_id = 'camera_link'
    camInfoMsgs.width = frameShape[0]
    camInfoMsgs.height = frameShape[1]
    # Fill in the camera intrinsics (fx, fy, cx, cy)
    camInfoMsgs.K = cameraMatrix.flatten()
    # No distortion for RealSense cameras
    camInfoMsgs.D = distCoeffs.flatten()
    # Return it
    return camInfoMsgs


# Run the main function
if __name__ == '__main__':
    main()
