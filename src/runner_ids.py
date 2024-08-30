#!/usr/bin/env python3

# import os
import rospy
# import rospkg
# import cv2 as cv
# import numpy as np
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from src.csr_sensors.sensors import sensorIDS
# from src.csr_detector.process import processStereoFrames
# from utils.valueParser import thresholdParser, channelParser


def main():
    # Initializing a ROS node
    rospy.init_node('csr_detector_idsCam')
    # rate = rospy.Rate(10)  # Publishing rate in Hz
    # publisherMask = rospy.Publisher('result_mask', Image, queue_size=10)
    # publisherCamL = rospy.Publisher('left_camera', Image, queue_size=10)
    # publisherCamR = rospy.Publisher('right_camera', Image, queue_size=10)
    # publisherResult = rospy.Publisher('result_frame', Image, queue_size=10)

    # # ROS Bridge
    # bridge = CvBridge()

    # # Loading configuration values
    # try:
    #     configs = rospy.get_param("~configs")
    #     homography = rospy.get_param("~homographyList")
    # except:
    #     rospy.logerr("No Config file found! Exiting ...")
    #     exit()

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
    # cap1 = sensorIDS.idsCamera(0)
    # cap2 = sensorIDS.idsCamera(1)

    # # Calibration
    # # The path to the current Python file
    # currentFilePath = os.path.abspath(__file__)
    # calibrationFilePath = os.path.join(os.path.dirname(
    #     currentFilePath), configs['preProcessing']['sensorProjectConfigPath'])
    # cap1.getCalibrationConfig(calibrationFilePath, 'cam1')
    # cap2.getCalibrationConfig(calibrationFilePath, 'cam2')

    # # Set ROI
    # roiDimension = configs['preProcessing']['roiDimension']
    # cap1.setROI(roiDimension['cap1']['x'], roiDimension['cap1']
    #             ['y'], roiDimension['cap1']['width'], roiDimension['cap1']['height'])
    # cap2.setROI(roiDimension['cap2']['x'], roiDimension['cap2']
    #             ['y'], roiDimension['cap2']['width'], roiDimension['cap2']['height'])

    # # Synchronization
    # cap1.syncAsMaster()
    # cap2.syncAsSlave()

    # # Capturing
    # cap1.startAquisition()
    # cap2.startAquisition()

    # # Exposure
    # cap1.setExposureTime(configs['sensor']['exposureTime'])
    # cap2.setExposureTime(configs['sensor']['exposureTime'])

    # while not rospy.is_shutdown():

    #     # Frames acquisition
    #     frame1 = cap1.getFrame()
    #     frame2 = cap2.getFrame()

    #     retL = False if (not np.any(frame1)) else True
    #     retR = False if (not np.any(frame2)) else True

    #     # Check if both cameras are connected
    #     if (not (retL and retR)):
    #         rospy.logerr("No connected devices found! Exiting ...")
    #         exit()

    #     # Change brightness
    #     frameL = cv.convertScaleAbs(
    #         frame1, alpha=configs['sensor']['brightness']['alpha'], beta=configs['sensor']['brightness']['beta'])
    #     frameR = cv.convertScaleAbs(
    #         frame2, alpha=configs['sensor']['brightness']['alpha'], beta=configs['sensor']['brightness']['beta'])

    #     # Flip the right frame
    #     frameR = cv.flip(frameR, 1)

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

    # cap1.closeLibrary()
    # cap2.closeLibrary()


# Run the main function
if __name__ == '__main__':
    main()
