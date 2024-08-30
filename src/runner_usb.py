#!/usr/bin/env python3

import rospy
import cv2 as cv
# import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.readConfig import readConfig
import csr_sensors.sensors.sensorUSB as usb
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

    # Camera
    capL = usb.createCameraObject(cfgUsbCam['ports']['lCam'])
    capR = usb.createCameraObject(cfgUsbCam['ports']['rCam'])

    if cfgGeneral['fpsBoost']:
        capL.set(cv.CAP_PROP_FPS, 30.0)
        capR.set(cv.CAP_PROP_FPS, 30.0)

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

        # Continue publishing
        rate.sleep()

    # Stop the pipeline
    capL.release()
    capR.release()
    rospy.loginfo(f'Framework stopped! [Double Vision USB Cameras Setup]')


# Run the main function
if __name__ == '__main__':
    main()
