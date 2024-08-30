#!/usr/bin/env python3

import rospy
from utils.readConfig import readConfig
# import cv2 as cv
# import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# import src.csr_sensors.sensors.sensorUSB as usb
# from src.csr_detector.process import processStereoFrames
# from utils.valueParser import thresholdParser, channelParser


def main():
    # Initializing a ROS node
    rospy.init_node('iMarker_detector_usb', anonymous=True)

    # Loading configuration values
    config = readConfig()
    if config is None:
        exit()

    # Get the config values
    cfgGui = config['gui']
    cfgMode = config['mode']
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
    pubMaskApplied = rospy.Publisher('mask_applied_img', Image, queue_size=10)

    # ROS Bridge
    bridge = CvBridge()

    # # Prepare the basic parameters
    # try:
    #     isThreshOts, isThreshBoth, isThreshBin = thresholdParser(
    #         configs['postProcessing']['thresholdMethod'])
    #     isRChannel, isGChannel, isBChannel, isAllChannels = channelParser(
    #         configs['preProcessing']['channel'])
    #     homographyMat = np.array(homography[configs['preProcessing']
    #                                         ['homographyMat']])
    #     # Prepare parameters based on what processor needs
    #     params = {
    #         'rChannel': isRChannel,
    #         'gChannel': isGChannel,
    #         'bChannel': isBChannel,
    #         'threshbin': isThreshBin,
    #         'threshots': isThreshOts,
    #         'threshboth': isThreshBoth,
    #         'allChannels': isAllChannels,
    #         'homographyMat': homographyMat,
    #         'windowWidth': configs['gui']['windowWidth'],
    #         'maxFeatures': configs['processing']['maxFeatures'],
    #         'threshold': configs['postProcessing']['threshold'],
    #         'preAligment': configs['processing']['preAligment'],
    #         'isMarkerLeftHanded': configs['markers']['leftHanded'],
    #         'erosionKernel': configs['postProcessing']['erodeKernelSize'],
    #         'enableCircularMask': configs['processing']['enableCircularROI'],
    #         'goodMatchPercentage': configs['processing']['goodMatchPercentage'],
    #         'gaussianKernel': configs['postProcessing']['gaussianBlurKernelSize'],
    #         'circlularMaskCoverage': configs['processing']['circlularMaskCoverage'],
    #     }
    # except:
    #     rospy.logerr("Error in fetching parameters! Exiting ...")
    #     exit()

    # # Camera
    # capL = usb.createCameraObject(configs['sensor']['usbCamPorts']['lCam'])
    # capR = usb.createCameraObject(configs['sensor']['usbCamPorts']['rCam'])

    # if configs['preProcessing']['fpsBoost']:
    #     capL.set(cv.CAP_PROP_FPS, 30.0)
    #     capR.set(cv.CAP_PROP_FPS, 30.0)

    # while not rospy.is_shutdown():
    #     # Retrieve frames
    #     # Note: if each of the cameras not working, retX will be False
    #     retL, frameL = usb.grabImage(capL)
    #     retR, frameR = usb.grabImage(capR)

    #     # Check if both cameras are connected
    #     if (not (retL and retR)):
    #         rospy.logerr("No connected devices found! Exiting ...")
    #         exit()

    #     # Change brightness
    #     frameL = cv.convertScaleAbs(
    #         frameL, alpha=configs['sensor']['brightness']['alpha'], beta=configs['sensor']['brightness']['beta'])
    #     frameR = cv.convertScaleAbs(
    #         frameR, alpha=configs['sensor']['brightness']['alpha'], beta=configs['sensor']['brightness']['beta'])

    #     # Flip the right frame
    #     if (configs['processing']['flipImage']):
    #         frameR = cv.flip(frameR, 1)

    #     # Convert to ROS
    #     frameLRos = bridge.cv2_to_imgmsg(frameL, "bgr8")
    #     frameRRos = bridge.cv2_to_imgmsg(frameR, "bgr8")
    #     publisherCamL.publish(frameLRos)
    #     publisherCamR.publish(frameRRos)

    #     # Process frames
    #     frame, mask = processStereoFrames(frameL, frameR, retL, retR, params)

    #     # Convert to ROS
    #     maskRos = bridge.cv2_to_imgmsg(mask, "bgr8")
    #     resultRos = bridge.cv2_to_imgmsg(frame, "bgr8")
    #     publisherMask.publish(maskRos)
    #     publisherResult.publish(resultRos)

    #     # Continue publishing
    #     rate.sleep()

    # capL.release()
    # capR.release()

    rospy.loginfo(f'Framework stopped! [Double Vision USB Cameras Setup]')


# Run the main function
if __name__ == '__main__':
    main()
