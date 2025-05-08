"""
📝 'iMarker Detector (ROS-based)' Software
    SPDX-FileCopyrightText: (2025) University of Luxembourg
    © 2025 University of Luxembourg
    Developed by: Ali TOURANI et al. at SnT / ARG.

'iMarker Detector (ROS-based)' is licensed under the "SNT NON-COMMERCIAL" License.
You may not use this file except in compliance with the License.
"""

def readConfig(self) -> dict:
    """
    Reads the configuration file available in the config folder.

    Returns:
    ----------
    config: dict
        The configuration dictionary containing the settings.
    """
    # Variables
    config = {}
    self.get_logger().info("Loading configuration values ...", once=True)

    try:
        # Declare parameters
        self.declare_parameter('mode.runner', "offvid")
        self.declare_parameter('mode.temporalSubtraction', False)
        self.declare_parameter('sensor.general.fpsBoost', False)
        self.declare_parameter('sensor.brightness.alpha', 1.0)
        self.declare_parameter('sensor.brightness.beta', 1.0)
        # Single-vision offline rosbag
        self.declare_parameter('sensor.offline.rosbag.rawImageTopic', "/camera/color/image_raw")
        # Dual-vision USB camera
        self.declare_parameter('sensor.usbCam.maskSize', 0.8)
        self.declare_parameter('sensor.usbCam.flipImage', True)
        self.declare_parameter('sensor.usbCam.enableMask', False)
        self.declare_parameter('sensor.usbCam.ports.lCam', "/dev/video4")
        self.declare_parameter('sensor.usbCam.ports.rCam', "/dev/video6")
        # Dual-vision iDS camera
        self.declare_parameter('sensor.ids.exposureTime', 20000)
        self.declare_parameter('sensor.ids.roi.cap1.x', 520)
        self.declare_parameter('sensor.ids.roi.cap1.y', 300)
        self.declare_parameter('sensor.ids.roi.cap1.width', 976)
        self.declare_parameter('sensor.ids.roi.cap1.height', 900)
        self.declare_parameter('sensor.ids.roi.cap2.x', 480)
        self.declare_parameter('sensor.ids.roi.cap2.y', 284)
        self.declare_parameter('sensor.ids.roi.cap2.width', 976)
        self.declare_parameter('sensor.ids.roi.cap2.height', 900)
        # Single-vision RealSense camera
        self.declare_parameter('sensor.realSense.fps', 30)
        self.declare_parameter('sensor.realSense.resolution.width', 640)
        self.declare_parameter('sensor.realSense.resolution.height', 480)
        # Algorithm parameters
        self.declare_parameter('algorithm.process.alignment.matchRate', 0.4)
        self.declare_parameter('algorithm.process.alignment.usePreset', True)
        self.declare_parameter('algorithm.process.alignment.maxFeatures', 500)
        self.declare_parameter('algorithm.process.subtractRL', True)
        self.declare_parameter('algorithm.process.channel', "g")  # ["b", "g", "r", "all"]
        self.declare_parameter('algorithm.process.colorRange.hsv_green.lower', [35, 120, 50])  # [35, 120, 50]
        self.declare_parameter('algorithm.process.colorRange.hsv_green.upper', [85, 255, 255])  # [85, 255, 255]
        self.declare_parameter('algorithm.postprocess.threshold.size', 55)
        self.declare_parameter('algorithm.postprocess.threshold.method', "binary")  # ["binary", "otsu", "adaptive"]
        self.declare_parameter('algorithm.postprocess.invertBinary', True)
        self.declare_parameter('algorithm.postprocess.erosionKernel', 1)
        self.declare_parameter('algorithm.postprocess.gaussianKernel', 1)
        # Marker parameters
        self.declare_parameter('marker.size', 0.07)
        self.declare_parameter('marker.detection.dictionary', "DICT_6X6_250")

        # Load the configuration file
        config = {
            'mode': {
                'runner': self.get_parameter('mode.runner').value,
                'temporalSubtraction': self.get_parameter('mode.temporalSubtraction').value
            },
            'sensor': {
                'general': {
                    'fpsBoost': self.get_parameter('sensor.general.fpsBoost').value,
                    'brightness': {
                        'alpha': self.get_parameter('sensor.brightness.alpha').value,
                        'beta': self.get_parameter('sensor.brightness.beta').value
                    }
                },
                'offline': {
                    'rosbag': {
                        'raw_image_topic': self.get_parameter('sensor.offline.rosbag.rawImageTopic').value
                    }
                },
                'usbCam': {
                    'maskSize': self.get_parameter('sensor.usbCam.maskSize').value,
                    'flipImage': self.get_parameter('sensor.usbCam.flipImage').value,
                    'enableMask': self.get_parameter('sensor.usbCam.enableMask').value,
                    'ports': {
                        'lCam': self.get_parameter('sensor.usbCam.ports.lCam').value,
                        'rCam': self.get_parameter('sensor.usbCam.ports.rCam').value
                    }
                },
                'ids': {
                    'exposureTime': self.get_parameter('sensor.ids.exposureTime').value,
                    'roi': {
                        'cap1': {
                            'x': self.get_parameter('sensor.ids.roi.cap1.x').value,
                            'y': self.get_parameter('sensor.ids.roi.cap1.y').value,
                            'width': self.get_parameter('sensor.ids.roi.cap1.width').value,
                            'height': self.get_parameter('sensor.ids.roi.cap1.height').value
                        },
                        'cap2': {
                            'x': self.get_parameter('sensor.ids.roi.cap2.x').value,
                            'y': self.get_parameter('sensor.ids.roi.cap2.y').value,
                            'width': self.get_parameter('sensor.ids.roi.cap2.width').value,
                            'height': self.get_parameter('sensor.ids.roi.cap2.height').value
                        }
                    }
                },
                'realSense': {
                    'fps': self.get_parameter('sensor.realSense.fps').value,
                    'resolution': {
                        'width': self.get_parameter(
                            'sensor.realSense.resolution.width').value,
                        'height': self.get_parameter(
                            'sensor.realSense.resolution.height').value
                    }
                }
            },
            'algorithm': {
                'process': {
                    'alignment': {
                        'matchRate': self.get_parameter(
                            'algorithm.process.alignment.matchRate').value,
                        'usePreset': self.get_parameter(
                            'algorithm.process.alignment.usePreset').value,
                        'maxFeatures': self.get_parameter(
                            'algorithm.process.alignment.maxFeatures').value
                    },
                    'subtractRL': self.get_parameter(
                        'algorithm.process.subtractRL').value,
                    'channel': self.get_parameter(
                        'algorithm.process.channel').value,
                    'colorRange': {
                        'hsv_green': {
                            'lower': self.get_parameter(
                                'algorithm.process.colorRange.hsv_green.lower').value,
                            'upper': self.get_parameter(
                                'algorithm.process.colorRange.hsv_green.upper').value
                        }
                    }
                },
                'postprocess': {
                    'threshold': {
                        'size': self.get_parameter(
                            'algorithm.postprocess.threshold.size').value,
                        'method': self.get_parameter(
                            'algorithm.postprocess.threshold.method').value
                    },
                    'invertBinary': self.get_parameter(
                        'algorithm.postprocess.invertBinary').value,
                    'erosionKernel': self.get_parameter(
                        'algorithm.postprocess.erosionKernel').value,
                    'gaussianKernel': self.get_parameter(
                        'algorithm.postprocess.gaussianKernel').value
                }
            },
            'marker': {
                'size': self.get_parameter('marker.size').value,
                'dictionary': self.get_parameter('marker.detection.dictionary').value
            }
        }

        # Check if the config is empty
        if not config:
            self.get_logger().error("Configuration file is empty! Exiting ...", once=True)
            return None

        # Return the config
        self.get_logger().info("Configuration values loaded successfully!\n")
        return config
    except Exception as e:
        self.get_logger().error(f"No configuration file found! {e} Exiting ...", once=True)
        return None
