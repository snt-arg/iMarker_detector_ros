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
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.readConfig import readConfig
import iMarker_sensors.sensors.usb_interface as usb
from marker_detector.arucoDetector import arucoDetector
from iMarker_algorithms.process import stereoFrameProcessing
from ament_index_python.packages import get_package_share_directory

class IMarkerDetector(Node):
    def __init__(self: 'IMarkerDetector') -> None:
        super().__init__('iMarker_detector_usb')
        self.bridge = CvBridge()

        # Load the configuration file
        self.configs = readConfig(self)
        if self.configs is None:
            self.get_logger().error("Failed to load configuration file.")
            return
        
        # Get the config values
        self.cfgMarker = self.configs['marker']
        self.cfgUsbCam = self.configs['sensor']['usbCam']
        self.cfgGeneral = self.configs['sensor']['general']

        # Variables

        # Inform the user
        self.get_logger().info('Framework started! [Dual-Vision USB Cameras Setup]')

        # Create publishers
        self.pubMask = self.create_publisher(Image, 'mask_img', 10)
        self.pubMarker = self.create_publisher(Image, 'marker_img', 10)
        self.pubRawL = self.create_publisher(Image, 'raw_img_left', 10)
        self.pubRawR = self.create_publisher(Image, 'raw_img_right', 10)

        # Prepare a notFound image
        pkgDir = get_package_share_directory('imarker_detector_ros')
        notFoundImagePath = os.path.join(pkgDir, 'src/notFound.png')
        self.notFoundImage = cv.imread(notFoundImagePath, cv.IMREAD_COLOR)

        # Camera
        self.capL = usb.createCameraObject(self.cfgUsbCam['ports']['lCam'])
        self.capR = usb.createCameraObject(self.cfgUsbCam['ports']['rCam'])

        # Check if the boost is enabled
        if self.cfgGeneral['fpsBoost']:
            self.capL.set(cv.CAP_PROP_FPS, 30.0)
            self.capR.set(cv.CAP_PROP_FPS, 30.0)
        
        # Create a timer to periodically read frames
        clock = 0.1  # seconds
        self.timer = self.create_timer(clock, self.process_frame)
    
    def process_frame(self):
        # Retrieve frames
        retL, frameLRaw = usb.grabImage(self.capL)
        retR, frameRRaw = usb.grabImage(self.capR)

        # Check if both cameras are connected
        if not retL and not retR:
            self.get_logger().error("No camera is connected! Exiting ...")
            return
        
        # Flip the right frame
        if (self.cfgUsbCam['flipImage']):
            frameRRaw = cv.flip(frameRRaw, 1)

        # Change brightness
        beta = float(self.cfgGeneral['brightness']['beta'])
        alpha = float(self.cfgGeneral['brightness']['alpha'])
        frameLRaw = cv.convertScaleAbs(frameLRaw, alpha=alpha, beta=beta)
        frameRRaw = cv.convertScaleAbs(frameRRaw, alpha=alpha, beta=beta)

        # Process frames
        frameL, frameR, frameMask = stereoFrameProcessing(
            frameLRaw, frameRRaw, retL, retR, self.configs, True)
        
        # Convert to RGB
        frameMask = cv.cvtColor(frameMask, cv.COLOR_GRAY2BGR)

        # Preparing the frames
        frameLRaw = frameLRaw if retL else self.notFoundImage
        frameRRaw = frameRRaw if retR else self.notFoundImage
        frameMask = frameMask if (retR and retL) else self.notFoundImage
        frameLRos = self.bridge.cv2_to_imgmsg(frameLRaw, "bgr8")
        frameRRos = self.bridge.cv2_to_imgmsg(frameRRaw, "bgr8")
        frameMaskRos = self.bridge.cv2_to_imgmsg(frameMask, "bgr8")

        # ArUco marker detection
        frameMarker = arucoDetector(
            frameMask, self.cfgMarker['detection']['dictionary'])
        frameMarker = frameMarker if (retR and retL) else self.notFoundImage
        frameMarkerRos = self.bridge.cv2_to_imgmsg(frameMarker, "bgr8")

        # Publishing the frames
        self.pubRawL.publish(frameLRos)
        self.pubRawR.publish(frameRRos)
        self.pubMask.publish(frameMaskRos)
        self.pubMarker.publish(frameMarkerRos)
    
    def destroy_node(self):
        # Stop the pipeline
        self.capL.release()
        self.capR.release()
        self.get_logger().info('Framework stopped! [Dual-Vision USB Cameras Setup]')
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
