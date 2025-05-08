#!/usr/bin/env python3

"""
📝 'iMarker Detector (ROS-based)' Software
    SPDX-FileCopyrightText: (2025) University of Luxembourg
    © 2025 University of Luxembourg
    Developed by: Ali TOURANI et al. at SnT / ARG.

'iMarker Detector (ROS-based)' is licensed under the "SNT NON-COMMERCIAL" License.
You may not use this file except in compliance with the License.
"""

import os
import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from utils.readConfig import readConfig
from sensor_msgs.msg import Image, CameraInfo
from iMarker_sensors.sensors import rs_interface
from utils.createRosMessage import getCameraInfo
from marker_detector.arucoDetector import arucoDetector
from ament_index_python.packages import get_package_share_directory
from iMarker_algorithms.process import sequentialFrameProcessing, singleFrameProcessing

class IMarkerDetector(Node):
    def __init__(self: 'IMarkerDetector') -> None:
        super().__init__('iMarker_detector_rs')
        self.bridge = CvBridge()

        # Load the configuration file
        self.configs = readConfig(self)
        if self.configs is None:
            self.get_logger().error("Failed to load configuration file.")
            return
        
        # Get the config values
        self.cfgMode = self.configs['mode']
        self.cfgMarker = self.configs['marker']
        self.cfgRS = self.configs['sensor']['realSense']
        self.cfgGeneral = self.configs['sensor']['general']

        # Variables
        self.prevFrame = None
        self.frameMask = None
        self.frameMaskApplied = None

        # Inform the user
        self.setupVariant = "Sequential Subtraction" if self.cfgMode[
            'temporalSubtraction'] else "Masking"
        self.get_logger().info(
            f'Framework started! [RealSense Single Vision Setup - {self.setupVariant}]')

        # Create publishers
        self.pubRaw = self.create_publisher(Image, 'raw_image', 10)
        self.pubMask = self.create_publisher(Image, 'mask_image', 10)
        self.pubMarker = self.create_publisher(Image, 'marker_image', 10)
        self.pubMaskApplied = self.create_publisher(Image, 'mask_applied_image', 10)
        self.pubCameraParams = self.create_publisher(
            CameraInfo, 'rs_cam_params', 10)
        
        # Create an object
        resolution = (self.cfgRS['resolution']['width'], self.cfgRS['resolution']['height'])
        self.rs = rs_interface.rsCamera(resolution, self.cfgRS['fps'])

        # Create a pipeline
        self.rs.createPipeline()

        # Prepare a notFound image
        pkgDir = get_package_share_directory('imarker_detector_ros')
        notFoundImagePath = os.path.join(pkgDir, 'src/notFound.png')
        self.notFoundImage = cv.imread(notFoundImagePath, cv.IMREAD_COLOR)

        # Start the pipeline
        self.isPipelineStarted = self.rs.startPipeline()

        # Create a timer to periodically read frames
        clock = 0.1  # seconds
        self.timer = self.create_timer(clock, self.process_frame)
    
    def process_frame(self):
        # Check if the frames are valid
        if not self.isPipelineStarted:
            return
        
        # Wait for the next frames from the camera
        frames = self.rs.grabFrames()
        ret = False if frames is None else True

        # Check if the frames are valid
        if not ret:
            self.get_logger().error("No frame captured! Exiting ...")
            return
        
        # Get the color frame
        currFrame, cameraMatrix, distCoeffs = self.rs.getColorFrame(frames)

        # Change brightness
        beta = float(self.cfgGeneral['brightness']['beta'])
        alpha = float(self.cfgGeneral['brightness']['alpha'])
        currFrame = cv.convertScaleAbs(currFrame, alpha=alpha, beta=beta)

        # Only the first time, copy the current frame to the previous frame
        if self.prevFrame is None:
            self.prevFrame = np.copy(currFrame)

        # Process frames
        if (self.cfgMode['temporalSubtraction']):
            # Process the frames
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
        
        # Camera Params
        camInfoMsgs = getCameraInfo(currFrame.shape, cameraMatrix, distCoeffs)

        # Convert to RGB
        frameMask = cv.cvtColor(frameMask, cv.COLOR_GRAY2BGR)

        # Preparing the frames
        frameRaw = currFrame if ret else self.notFoundImage
        frameMask = frameMask if ret else self.notFoundImage
        frameMaskApplied = frameMaskApplied if ret else self.notFoundImage
        frameRawRos = self.bridge.cv2_to_imgmsg(frameRaw, "bgr8")
        frameMaskRos = self.bridge.cv2_to_imgmsg(frameMask, "bgr8")
        frameMaskApplied = self.bridge.cv2_to_imgmsg(frameMaskApplied, "bgr8")

        # ArUco marker detection
        frameMarker = arucoDetector(
            frameMask, cameraMatrix, distCoeffs, self.cfgMarker['detection']['dictionary'],
            self.cfgMarker['structure']['size'])
        frameMarker = frameMarker if ret else self.notFoundImage
        frameMarkerRos = self.bridge.cv2_to_imgmsg(frameMarker, "bgr8")

        # Publishing the frames
        self.pubRaw.publish(frameRawRos)
        self.pubMask.publish(frameMaskRos)
        self.pubMarker.publish(frameMarkerRos)
        self.pubCameraParams.publish(camInfoMsgs)
        self.pubMaskApplied.publish(frameMaskApplied)

        # Save the previous frame
        self.prevFrame = np.copy(currFrame)
    
    def destroy_node(self):
        if self.isPipelineStarted:
            self.rs.stopPipeline()
        self.get_logger().info(
            f'Framework stopped! [RealSense Single Vision Setup - {self.setupVariant}]')
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
