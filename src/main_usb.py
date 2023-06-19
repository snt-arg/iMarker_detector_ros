#!/usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge
import src.csr_sensors.sensors.sensorUSB as usb
from src.csr_detector.process import processFrames
# from config import ports, fpsBoost, flipImage, preAligment, homographyMat, windowWidth


def main():
    # Initializing a ROS node
    rospy.init_node('csr_detector_usbCam')

    # Loading configuration values
    try:
        configs = rospy.get_param("~config")
    except:
        rospy.logerr("No Config file found!")

    print()
    # Camera
    capL = usb.createCameraObject(configs['sensor']['usbCamPorts']['lCam'])
    capR = usb.createCameraObject(configs['sensor']['usbCamPorts']['rCam'])

    if configs['preProcessing']['fpsBoost']:
        capL.set(cv.CAP_PROP_FPS, 30.0)
        capR.set(cv.CAP_PROP_FPS, 30.0)

    while True:
        # Retrieve frames
        # Note: if each of the cameras not working, retX will be False
        retL, frameL = usb.grabImage(capL)
        retR, frameR = usb.grabImage(capR)

    #     # Get the values from the GUI
    #     params = {'maxFeatures': values['MaxFeat'], 'goodMatchPercentage': values['MatchRate'],
    #               'circlularMaskCoverage': values['CircMask'], 'threshold': values['Threshold'],
    #               'erosionKernel': values['Erosion'], 'gaussianKernel': values['Gaussian'],
    #               'enableCircularMask': values['CircMaskEnable'], 'allChannels': values['AChannels'],
    #               'rChannel': values['RChannel'], 'gChannel': values['GChannel'], 'bChannel': values['BChannel'],
    #               'threshboth': values['ThreshBoth'], 'threshbin': values['ThreshBin'],
    #               'threshots': values['ThreshOts'], 'isMarkerLeftHanded': values['MarkerLeftHanded'],
    #               'preAligment': preAligment, 'homographyMat': homographyMat, 'windowWidth': windowWidth
    #               }

    #     # Change brightness
    #     frameL = cv.convertScaleAbs(
    #         frameL, alpha=values['camAlpha'], beta=values['camBeta'])
    #     frameR = cv.convertScaleAbs(
    #         frameR, alpha=values['camAlpha'], beta=values['camBeta'])

    #     # Flip the right frame
    #     if (flipImage):
    #         frameR = cv.flip(frameR, 1)

    #     # Process frames
    #     frame, mask = processFrames(frameL, frameR, retL, retR, params)

    #     # Show the frames
    #     frame = cv.imencode(".png", frame)[1].tobytes()

    capL.release()
    capR.release()


# Run the program
main()
