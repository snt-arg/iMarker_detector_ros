#!/usr/bin/env python3

import rospy
# import cv2 as cv
# import numpy as np
# from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
# from src.csr_sensors.sensors import sensorRealSense
# from utils.valueParser import thresholdParser, channelParser
# from src.csr_detector.process import processSequentialFrames, processSingleFrame


def main():
    # Initializing a ROS node
    rospy.init_node('csr_detector_rsCam')
    # rate = rospy.Rate(10)  # Publishing rate in Hz
    # publisherCam = rospy.Publisher('main_camera', Image, queue_size=10)
    # publisherMask = rospy.Publisher('result_mask', Image, queue_size=10)
    # publisherResult = rospy.Publisher('result_frame', Image, queue_size=10)
    # publisherCamParam = rospy.Publisher(
    #     'rs_camera_params', CameraInfo, queue_size=10)

#     # Previous frame
#     prevFrame = None

#     # ROS Bridge
#     bridge = CvBridge()

#     # Loading configuration values
#     try:
#         configs = rospy.get_param("~configs")
#     except:
#         rospy.logerr("No Config file found! Exiting ...")
#         exit()

#     # Prepare the basic parameters
#     try:
#         isThreshOts, isThreshBoth, isThreshBin = thresholdParser(
#             configs['postProcessing']['thresholdMethod'])
#         isRChannel, isGChannel, isBChannel, isAllChannels = channelParser(
#             configs['preProcessing']['channel'])
#         # Prepare parameters based on what processor needs
#         params = {
#             'rChannel': isRChannel,
#             'gChannel': isGChannel,
#             'bChannel': isBChannel,
#             'threshbin': isThreshBin,
#             'threshots': isThreshOts,
#             'threshboth': isThreshBoth,
#             'allChannels': isAllChannels,
#             'windowWidth': configs['gui']['windowWidth'],
#             'threshold': configs['postProcessing']['threshold'],
#             'isMarkerLeftHanded': configs['markers']['leftHanded'],
#             'erosionKernel': configs['postProcessing']['erodeKernelSize'],
#             'invertBinaryImage': configs['processing']['invertBinaryImage'],
#             'enableCircularMask': configs['processing']['enableCircularROI'],
#             'gaussianKernel': configs['postProcessing']['gaussianBlurKernelSize'],
#         }
#     except:
#         rospy.logerr("Error in fetching parameters! Exiting ...")
#         exit()

#     monoSetupVariant = "Sequential Subtraction" if configs[
#         'processing']['isSequentialSubtraction'] else "Thresholding"
#     print(f'\n[RealSense Mono Setup - {monoSetupVariant}]\n')

#     # Camera
#     realSenseResolution = (configs['sensor']['realSenseResolution']['width'],
#                            configs['sensor']['realSenseResolution']['height'])
#     rs = sensorRealSense.rsCamera(
#         realSenseResolution, configs['sensor']['realSenseFps'])

#     # Create a pipeline
#     rs.createPipeline()

#     # Start the pipeline
#     rs.startPipeline()

#     try:
#         while not rospy.is_shutdown():

#             # Wait for the next frames from the camera
#             frames = rs.grabFrames()

#             # Get the color frame
#             colorFrame, colorCamIntrinsics = rs.getColorFrame(frames)

#             # Change brightness
#             colorFrame = cv.convertScaleAbs(
#                 colorFrame, alpha=configs['sensor']['brightness']['alpha'],
#                 beta=configs['sensor']['brightness']['beta'])

#             if prevFrame is None:
#                 prevFrame = np.copy(colorFrame)

#             # Convert to ROS
#             frameRos = bridge.cv2_to_imgmsg(colorFrame, "bgr8")
#             publisherCam.publish(frameRos)

#             # Process frames
#             if (configs['processing']['isSequentialSubtraction']):
#                 frame, mask = processSequentialFrames(
#                     prevFrame, colorFrame, True, params)
#             else:
#                 frame, mask = processSingleFrame(colorFrame, True, params)

#             # Camera Params
#             # depthIntrinsics, colorIntrinsics = rs.getIntrinsicParams()
#             camInfoMsgs = getCameraInfo(colorCamIntrinsics)

#             # Convert to ROS
#             maskRos = bridge.cv2_to_imgmsg(mask, "bgr8")
#             resultRos = bridge.cv2_to_imgmsg(frame, "bgr8")
#             publisherMask.publish(maskRos)
#             publisherResult.publish(resultRos)
#             publisherCamParam.publish(camInfoMsgs)

#             # Save the previous frame
#             prevFrame = np.copy(colorFrame)

#             # Continue publishing
#             rate.sleep()

#     finally:
#         # Stop the pipeline and close the windows
#         rospy.logerr("Error in RealSense pipeline! Exiting ...")
#         rs.stopPipeline()


# def getCameraInfo(intrinsics):
#     # Create a message and fill it with calibration params
#     camInfoMsgs = CameraInfo()
#     camInfoMsgs.header.frame_id = 'camera_link'
#     camInfoMsgs.width = intrinsics.width
#     camInfoMsgs.height = intrinsics.height
#     # Fill in the camera intrinsics (fx, fy, cx, cy)
#     camInfoMsgs.K = [intrinsics.fx, 0, intrinsics.ppx,
#                      0, intrinsics.fy, intrinsics.ppy,
#                      0, 0, 1]
#     # No distortion for RealSense cameras
#     camInfoMsgs.D = [0, 0, 0, 0, 0]
#     # Return it
#     return camInfoMsgs


# Run the main function
if __name__ == '__main__':
    main()
