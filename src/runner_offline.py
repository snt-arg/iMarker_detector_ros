#!/usr/bin/env python3

import rospy
# import cv2 as cv
# import numpy as np
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# import src.csr_sensors.sensors.sensorUSB as usb
# from src.csr_detector.process import processSingleFrame
# from utils.valueParser import thresholdParser, channelParser


def main():
    # Initializing a ROS node
    rospy.init_node('csr_detector_usbCamMono')
    # rate = rospy.Rate(10)  # Publishing rate in Hz
    # publisherCam = rospy.Publisher('main_camera', Image, queue_size=10)
    # publisherMask = rospy.Publisher('result_mask', Image, queue_size=10)
    # publisherResult = rospy.Publisher('result_frame', Image, queue_size=10)

    # # ROS Bridge
    # bridge = CvBridge()

    # # Loading configuration values
    # try:
    #     configs = rospy.get_param("~configs")
    # except:
    #     rospy.logerr("No Config file found! Exiting ...")
    #     exit()

    #     # Prepare the basic parameters
    # try:
    #     isThreshOts, isThreshBoth, isThreshBin = thresholdParser(
    #         configs['postProcessing']['thresholdMethod'])
    #     isRChannel, isGChannel, isBChannel, isAllChannels = channelParser(
    #         configs['preProcessing']['channel'])
    #     # Prepare parameters based on what processor needs
    #     params = {
    #         'rChannel': isRChannel,
    #         'gChannel': isGChannel,
    #         'bChannel': isBChannel,
    #         'threshbin': isThreshBin,
    #         'threshots': isThreshOts,
    #         'threshboth': isThreshBoth,
    #         'allChannels': isAllChannels,
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
    # cap = usb.createCameraObject(configs['sensor']['usbCamPorts']['lCam'])

    # if configs['preProcessing']['fpsBoost']:
    #     cap.set(cv.CAP_PROP_FPS, 30.0)

    # while not rospy.is_shutdown():
    #     # Retrieve frames
    #     ret, frame = usb.grabImage(cap)

    #     # Check if both cameras are connected
    #     if (not ret):
    #         rospy.logerr("No connected devices found! Exiting ...")
    #         exit()

    #     # Change brightness
    #     frame = cv.convertScaleAbs(
    #         frame, alpha=configs['sensor']['brightness']['alpha'], beta=configs['sensor']['brightness']['beta'])

    #     # Convert to ROS
    #     frameRos = bridge.cv2_to_imgmsg(frame, "bgr8")
    #     publisherCam.publish(frameRos)

    #     # Process frames
    #     frame, mask = processSingleFrame(frame, ret, params)

    #     # Convert to ROS
    #     maskRos = bridge.cv2_to_imgmsg(mask, "bgr8")
    #     resultRos = bridge.cv2_to_imgmsg(frame, "bgr8")
    #     publisherMask.publish(maskRos)
    #     publisherResult.publish(resultRos)

    #     # Continue publishing
    #     rate.sleep()

    # cap.release()


# Run the main function
if __name__ == '__main__':
    main()
