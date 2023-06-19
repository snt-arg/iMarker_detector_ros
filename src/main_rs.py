#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from src.csr_sensors.sensors import sensorRealSense
from src.csr_detector.vision.channelSeparator import channelSeparator
from config import realSenseResolution, realSenseFps, windowWidth


def main():
    # Initializing a ROS node
    rospy.init_node('csr_detector_rsCam')

    # Loading configuration values
    try:
        configs = rospy.get_param("~config")
    except:
        rospy.logerr("No Config file found!")

    print(configs)

    # rs = sensorRealSense.rsCamera(realSenseResolution, realSenseFps)

    # # Create a pipeline
    # rs.createPipeline()

    # # Start the pipeline
    # rs.startPipeline()

    # try:
    #     while True:
    #         event, values = window.read(timeout=10)

    #         # Wait for the next frames from the camera
    #         frames = rs.grabFrames()

    #         # Get the color frame
    #         colorFrame = rs.getColorFrame(frames)

    #         # Get the values from the GUI
    #         params = {'allChannels': values['AChannels'], 'rChannel': values['RChannel'],
    #                   'gChannel': values['GChannel'], 'bChannel': values['BChannel'],
    #                   'isMarkerLeftHanded': values['MarkerLeftHanded'], 'windowWidth': windowWidth
    #                   }

    #         # Change brightness
    #         colorFrame = cv.convertScaleAbs(
    #             colorFrame, alpha=values['camAlpha'], beta=values['camBeta'])

    #         procFrame = channelSeparator(colorFrame, params)

    #         # Show the frames
    #         frame = cv.imencode(".png", procFrame)[1].tobytes()

    # finally:
    #     # Stop the pipeline and close the windows
    #     rs.stopPipeline()
    #     cv.destroyAllWindows()


# Run the program
main()
