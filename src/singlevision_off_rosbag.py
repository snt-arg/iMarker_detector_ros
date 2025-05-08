#!/usr/bin/env python3

"""
📝 'iMarker Detector (ROS-based)' Software
    SPDX-FileCopyrightText: (2025) University of Luxembourg
    © 2025 University of Luxembourg
    Developed by: Ali TOURANI et al. at SnT / ARG.

'iMarker Detector (ROS-based)' is licensed under the "SNT NON-COMMERCIAL" License.
You may not use this file except in compliance with the License.
"""

import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.readConfig import readConfig
from marker_detector.arucoDetector import arucoDetector
from iMarker_algorithms.process import sequentialFrameProcessing, singleFrameProcessing

class IMarkerDetector(Node):
    def __init__(self: 'IMarkerDetector') -> None:
        super().__init__('iMarker_detector_off')
        self.bridge = CvBridge()

        # Load the configuration file
        self.configs = readConfig(self)
        if self.configs is None:
            self.get_logger().error("Failed to load configuration file.")
            return

        # Get the config values
        self.cfgMode = self.configs['mode']
        self.cfgMarker = self.configs['marker']
        self.cfgGeneral = self.configs['sensor']['general']
        self.cfgOffline = self.configs['sensor']['offline']['rosbag']

        # Variables
        self.prevFrame = None
        self.frameMask = None
        self.frameMaskApplied = None

        # Inform the user
        self.setupVariant = "Sequential Subtraction" if self.cfgMode[
            'temporalSubtraction'] else "Masking"
        self.get_logger().info(
            f'Framework started! [Offline Rosbag Captured by Single Vision Setup - {self.setupVariant}]')
        
        # Create subscribers
        self.create_subscription(Image, self.cfgOffline['raw_image_topic'], self.fetch_raw_image, 10)

        # Create publishers
        self.pubRaw = self.create_publisher(Image, 'raw_image', 10)
        self.pubMask = self.create_publisher(Image, 'mask_image', 10)
        self.pubMarker = self.create_publisher(Image, 'marker_image', 10)
        self.pubMaskApplied = self.create_publisher(Image, 'mask_applied_image', 10)
    
    def fetch_raw_image(self, msg: Image) -> None:
        currFrame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Change brightness
        beta = float(self.cfgGeneral['brightness']['beta'])
        alpha = float(self.cfgGeneral['brightness']['alpha'])
        currFrame = cv.convertScaleAbs(currFrame, alpha=alpha, beta=beta)

        # Only the first time, copy the current frame to the previous frame
        if self.prevFrame is None:
            self.prevFrame = np.copy(currFrame)

        # Process frames
        if self.cfgMode['temporalSubtraction']:
            pFrame, cFrame, frameMask = sequentialFrameProcessing(
                self.prevFrame, currFrame, True, self.configs)
            # Apply the mask
            frameMaskApplied = cv.bitwise_and(
                cFrame, cFrame, mask=frameMask)
        else:
            # Process the frames
            cFrame, frameMask = singleFrameProcessing(
                currFrame, True, self.configs)
            # Apply the mask
            frameMaskApplied = cv.bitwise_and(
                cFrame, cFrame, mask=frameMask)

        # Convert the mask to RGB
        frameMask = cv.cvtColor(frameMask, cv.COLOR_GRAY2BGR)

        # Preparing the frames for publishing
        frameRawRos = self.bridge.cv2_to_imgmsg(currFrame, "bgr8")
        frameMaskRos = self.bridge.cv2_to_imgmsg(frameMask, "bgr8")
        frameMaskAppliedRos = self.bridge.cv2_to_imgmsg(frameMaskApplied, "bgr8")

        # ArUco marker detection
        frameMarker = arucoDetector(
            frameMask, self.cfgMarker['detection']['dictionary'])
        frameMarkerRos = self.bridge.cv2_to_imgmsg(frameMarker, "bgr8")

        # Publishing the frames
        self.pubRaw.publish(frameRawRos)
        self.pubMask.publish(frameMaskRos)
        self.pubMarker.publish(frameMarkerRos)
        self.pubMaskApplied.publish(frameMaskAppliedRos)

        # Save the previous frame
        self.prevFrame = np.copy(currFrame)
    
    def destroy_node(self):
        self.get_logger().info(
            f'Framework stopped! [Offline Rosbag Captured by Single Vision Setup - {self.setupVariant}]')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IMarkerDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

# Run the main function
if __name__ == '__main__':
    main()
