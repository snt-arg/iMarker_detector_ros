#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import src.csr_sensors.sensors.sensorUSB as usb
from src.csr_detector.process import processFrames
from utils.valueParser import thresholdParser, channelParser


def main():
    # Initializing a ROS node
    rospy.init_node('csr_detector_usbCam')
    rate = rospy.Rate(10)  # Publishing rate in Hz
    publisherMask = rospy.Publisher('result_mask', Image, queue_size=10)
    publisherCamL = rospy.Publisher('left_camera', Image, queue_size=10)
    publisherCamR = rospy.Publisher('right_camera', Image, queue_size=10)
    publisherResult = rospy.Publisher('result_frame', Image, queue_size=10)

    # ROS Bridge
    bridge = CvBridge()

    # Loading configuration values
    try:
        configs = rospy.get_param("~configs")
        homography = rospy.get_param("~homographyList")
    except:
        rospy.logerr("No Config file found! Exiting ...")
        exit()

    # Prepare the basic parameters
    try:
        isThreshOts, isThreshBoth, isThreshBin = thresholdParser(
            configs['postProcessing']['thresholdMethod'])
        isRChannel, isGChannel, isBChannel, isAllChannels = channelParser(
            configs['preProcessing']['channel'])
        homographyMat = np.array(homography[configs['preProcessing']
                                            ['homographyMat']])
        # Prepare parameters based on what processor needs
        params = {
            'rChannel': isRChannel,
            'gChannel': isGChannel,
            'bChannel': isBChannel,
            'threshbin': isThreshBin,
            'threshots': isThreshOts,
            'threshboth': isThreshBoth,
            'allChannels': isAllChannels,
            'homographyMat': homographyMat,
            'windowWidth': configs['gui']['windowWidth'],
            'maxFeatures': configs['processing']['maxFeatures'],
            'threshold': configs['postProcessing']['threshold'],
            'preAligment': configs['processing']['preAligment'],
            'isMarkerLeftHanded': configs['markers']['leftHanded'],
            'erosionKernel': configs['postProcessing']['erodeKernelSize'],
            'enableCircularMask': configs['processing']['enableCircularROI'],
            'goodMatchPercentage': configs['processing']['goodMatchPercentage'],
            'gaussianKernel': configs['postProcessing']['gaussianBlurKernelSize'],
            'circlularMaskCoverage': configs['processing']['circlularMaskCoverage'],
        }
    except:
        rospy.logerr("Error in fetching parameters! Exiting ...")
        exit()

    # Camera
    capL = usb.createCameraObject(configs['sensor']['usbCamPorts']['lCam'])
    capR = usb.createCameraObject(configs['sensor']['usbCamPorts']['rCam'])

    if configs['preProcessing']['fpsBoost']:
        capL.set(cv.CAP_PROP_FPS, 30.0)
        capR.set(cv.CAP_PROP_FPS, 30.0)

    while not rospy.is_shutdown():
        # Retrieve frames
        # Note: if each of the cameras not working, retX will be False
        retL, frameL = usb.grabImage(capL)
        retR, frameR = usb.grabImage(capR)

        # Check if both cameras are connected
        if (not (retL and retR)):
            rospy.logerr("No connected devices found! Exiting ...")
            exit()

        # Change brightness
        frameL = cv.convertScaleAbs(
            frameL, alpha=configs['sensor']['brightness']['alpha'], beta=configs['sensor']['brightness']['beta'])
        frameR = cv.convertScaleAbs(
            frameR, alpha=configs['sensor']['brightness']['alpha'], beta=configs['sensor']['brightness']['beta'])

        # Flip the right frame
        if (configs['processing']['flipImage']):
            frameR = cv.flip(frameR, 1)

        # Convert to ROS
        frameLRos = bridge.cv2_to_imgmsg(frameL, "bgr8")
        frameRRos = bridge.cv2_to_imgmsg(frameR, "bgr8")
        publisherCamL.publish(frameLRos)
        publisherCamR.publish(frameRRos)

        # Process frames
        frame, mask = processFrames(frameL, frameR, retL, retR, params)

        # Convert to ROS
        maskRos = bridge.cv2_to_imgmsg(mask, "bgr8")
        resultRos = bridge.cv2_to_imgmsg(frame, "bgr8")
        publisherMask.publish(maskRos)
        publisherResult.publish(resultRos)

        # Continue publishing
        rate.sleep()

    capL.release()
    capR.release()


# Run the program
main()
