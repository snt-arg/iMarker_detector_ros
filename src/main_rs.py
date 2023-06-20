#!/usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from src.csr_sensors.sensors import sensorRealSense
from utils.valueParser import thresholdParser, channelParser
from src.csr_detector.vision.channelSeparator import channelSeparator


def main():
    # Initializing a ROS node
    rospy.init_node('csr_detector_rsCam')
    rate = rospy.Rate(10)  # Publishing rate in Hz
    publisherCam = rospy.Publisher('main_camera', Image, queue_size=10)
    publisherMask = rospy.Publisher('result_mask', Image, queue_size=10)
    publisherResult = rospy.Publisher('result_frame', Image, queue_size=10)

    # ROS Bridge
    bridge = CvBridge()

    # Loading configuration values
    try:
        configs = rospy.get_param("~configs")
    except:
        rospy.logerr("No Config file found! Exiting ...")
        exit()

    # Prepare the basic parameters
    try:
        isThreshOts, isThreshBoth, isThreshBin = thresholdParser(
            configs['postProcessing']['thresholdMethod'])
        isRChannel, isGChannel, isBChannel, isAllChannels = channelParser(
            configs['preProcessing']['channel'])
        # Prepare parameters based on what processor needs
        params = {
            'rChannel': isRChannel,
            'gChannel': isGChannel,
            'bChannel': isBChannel,
            'threshbin': isThreshBin,
            'threshots': isThreshOts,
            'threshboth': isThreshBoth,
            'allChannels': isAllChannels,
            'windowWidth': configs['gui']['windowWidth'],
            'isMarkerLeftHanded': configs['markers']['leftHanded'],
        }
    except:
        rospy.logerr("Error in fetching parameters! Exiting ...")
        exit()

    # Camera
    realSenseResolution = (configs['sensor']['realSenseResolution']['width'],
                           configs['sensor']['realSenseResolution']['height'])
    rs = sensorRealSense.rsCamera(
        realSenseResolution, configs['sensor']['realSenseFps'])

    # Create a pipeline
    rs.createPipeline()

    # Start the pipeline
    rs.startPipeline()

    try:
        while not rospy.is_shutdown():

            # Wait for the next frames from the camera
            frames = rs.grabFrames()

            # Get the color frame
            colorFrame = rs.getColorFrame(frames)

            # Change brightness
            colorFrame = cv.convertScaleAbs(
                colorFrame, alpha=configs['sensor']['brightness']['alpha'],
                beta=configs['sensor']['brightness']['beta'])

            procFrame = channelSeparator(colorFrame, params)

            # Convert to ROS
            frameRos = bridge.cv2_to_imgmsg(procFrame, "bgr8")
            colorFrameRos = bridge.cv2_to_imgmsg(colorFrame, "bgr8")
            publisherResult.publish(frameRos)
            publisherCam.publish(colorFrameRos)

            # Continue publishing
            rate.sleep()

    finally:
        # Stop the pipeline and close the windows
        rospy.logerr("Error in RealSense pipeline! Exiting ...")
        rs.stopPipeline()


# Run the program
main()
