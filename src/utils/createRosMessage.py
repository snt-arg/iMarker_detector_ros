"""
📝 'iMarker Detector (ROS-based)' Software
    SPDX-FileCopyrightText: (2025) University of Luxembourg
    © 2025 University of Luxembourg
    Developed by: Ali TOURANI et al. at SnT / ARG.

'iMarker Detector (ROS-based)' is licensed under the "SNT NON-COMMERCIAL" License.
You may not use this file except in compliance with the License.
"""

from sensor_msgs.msg import CameraInfo


def getCameraInfo(frameShape, cameraMatrix, distCoeffs):
    """
    Create a ROS CameraInfo message from the camera parameters.

    Parameters
    ----------
    frameShape: tuple
        The shape of the frame (width, height).
    cameraMatrix: numpy.ndarray
        The camera matrix of the camera.
    distCoeffs: numpy.ndarray
        The distortion coefficients of the camera.

    Returns
    ----------
    camInfoMsgs: CameraInfo
        The CameraInfo message with the camera parameters.
    """
    # Create a message and fill it with calibration params
    camInfoMsgs = CameraInfo()
    camInfoMsgs.header.frame_id = 'camera_link'
    camInfoMsgs.width = frameShape[0]
    camInfoMsgs.height = frameShape[1]
    # Fill in the camera intrinsics (fx, fy, cx, cy)
    camInfoMsgs.K = cameraMatrix.flatten()
    # No distortion for RealSense cameras
    camInfoMsgs.D = distCoeffs.flatten()
    # Return it
    return camInfoMsgs
