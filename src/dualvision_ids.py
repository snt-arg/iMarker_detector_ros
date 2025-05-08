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
from sensor_msgs.msg import Image
from utils.readConfig import readConfig
from iMarker_sensors.sensors import ids_interface
from marker_detector.arucoDetector import arucoDetector
from iMarker_algorithms.process import stereoFrameProcessing
from ament_index_python.packages import get_package_share_directory
from iMarker_sensors.sensors.config.presets import homographyMatrixPreset_iDS

class IMarkerDetector(Node):
    def __init__(self: 'IMarkerDetector') -> None:
        super().__init__('iMarker_detector_dv_ids')
        self.bridge = CvBridge()

        # Load the configuration file
        self.configs = readConfig(self)
        if self.configs is None:
            self.get_logger().error("Failed to load configuration file.")
            return
        
        # Get the config values
        self.cfgMarker = self.configs['marker']
        self.cfgIDSCam = self.configs['sensor']['ids']
        self.cfgGeneral = self.configs['sensor']['general']

        # Variables

        # Inform the user
        self.get_logger().info('Framework started! [Dual-Vision iDS Cameras Setup]')

        # Create publishers
        self.pubMask = self.create_publisher(Image, 'mask_img', 10)
        self.pubMarker = self.create_publisher(Image, 'marker_img', 10)
        self.pubRawL = self.create_publisher(Image, 'raw_img_left', 10)
        self.pubRawR = self.create_publisher(Image, 'raw_img_right', 10)

        # Prepare a notFound image
        pkgDir = get_package_share_directory('imarker_detector_ros')
        notFoundImagePath = os.path.join(pkgDir, 'src/notFound.png')
        self.notFoundImage = cv.imread(notFoundImagePath, cv.IMREAD_COLOR)
        sensorsConfigPath = os.path.join(pkgDir, 'src/iMarker_sensors/sensors/config')

        # Fetch the cameras
        self.cap1 = ids_interface.idsCamera(0)
        self.cap2 = ids_interface.idsCamera(1)

        # Get the calibration configuration
        self.cap1.getCalibrationConfig(sensorsConfigPath, 'cam1')
        self.cap2.getCalibrationConfig(sensorsConfigPath, 'cam2')

        # Set the ROI
        width = self.cfgIDSCam['roi']['cap1']['width']
        height = self.cfgIDSCam['roi']['cap1']['height']
        self.cap1.setROI(self.cfgIDSCam['roi']['cap1']['x'], self.cfgIDSCam['roi']['cap1']
                    ['y'], width, height)
        self.cap2.setROI(self.cfgIDSCam['roi']['cap2']['x'], self.cfgIDSCam['roi']['cap2']
                    ['y'], width, height)

        # Synchronize the cameras
        self.cap1.syncAsMaster()
        self.cap2.syncAsSlave()

        # Capture the frames
        self.cap1.startAquisition()
        self.cap2.startAquisition()

        # Set the exposure time
        self.cap1.setExposureTime(self.cfgIDSCam['exposureTime'])
        self.cap2.setExposureTime(self.cfgIDSCam['exposureTime'])

        # Create a timer to periodically read frames
        clock = 0.1  # seconds
        self.timer = self.create_timer(clock, self.process_frame)
    
    def process_frame(self):
        # Retrieve frames
        frame1Raw = self.cap1.getFrame()
        frame2Raw = self.cap2.getFrame()

        retL = False if (not np.any(frame1Raw)) else True
        retR = False if (not np.any(frame2Raw)) else True

        if not (retL and retR):
            self.get_logger().error("No camera is connected! Exiting ...")
            return
        
        # Flip the right frame
        frame2Raw = cv.flip(frame2Raw, 1)

        # Change brightness
        beta = float(self.cfgGeneral['brightness']['beta'])
        alpha = float(self.cfgGeneral['brightness']['alpha'])
        frame1Raw = cv.convertScaleAbs(frame1Raw, alpha=alpha, beta=beta)
        frame2Raw = cv.convertScaleAbs(frame2Raw, alpha=alpha, beta=beta)

        # Add the homography matrix to the config
        self.configs['presetMat'] = homographyMatrixPreset_iDS

        # Process frames
        frame1, frame2, frameMask = stereoFrameProcessing(
            frame1Raw, frame2Raw, retL, retR, self.configs, False)

        # Convert to RGB
        frameMask = cv.cvtColor(frameMask, cv.COLOR_GRAY2BGR)

        # Preparing the frames
        frame1Raw = frame1Raw if retL else self.notFoundImage
        frame2Raw = frame2Raw if retR else self.notFoundImage
        frameMask = frameMask if (retR and retL) else self.notFoundImage
        frame1Ros = self.bridge.cv2_to_imgmsg(frame1Raw, "bgr8")
        frame2Ros = self.bridge.cv2_to_imgmsg(frame2Raw, "bgr8")
        frameMaskRos = self.bridge.cv2_to_imgmsg(frameMask, "bgr8")

        # ArUco marker detection
        frameMarker = arucoDetector(
            frameMask, self.cfgMarker['detection']['dictionary'])
        frameMarker = frameMarker if (retR and retL) else self.notFoundImage
        frameMarkerRos = self.bridge.cv2_to_imgmsg(frameMarker, "bgr8")

        # Publishing the frames
        self.pubRawL.publish(frame1Ros)
        self.pubRawR.publish(frame2Ros)
        self.pubMask.publish(frameMaskRos)
        self.pubMarker.publish(frameMarkerRos)
    
    def destroy_node(self):
        # Stop the cameras
        self.cap1.closeLibrary()
        self.cap2.closeLibrary()
        self.get_logger().info('Framework stopped! [Dual-Vision iDS Cameras Setup]')
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
    try:
        import ids_peak
        import ids_peak_ipl
        main()
    except ImportError:
        print(
            '[Error] Please install the `ids-peak` and `ids-peak-ipl` packages to use the iDS camera runner.')
