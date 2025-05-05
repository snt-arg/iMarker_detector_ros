"""
📝 'iMarker Detector (ROS-based)' Software
    SPDX-FileCopyrightText: (2025) University of Luxembourg
    © 2025 University of Luxembourg
    Developed by: Ali TOURANI et al. at SnT / ARG.

'iMarker Detector (ROS-based)' is licensed under the "SNT NON-COMMERCIAL" License.
You may not use this file except in compliance with the License.
"""

import rospy


def readConfig() -> dict:
    """
    Reads the configuration file available in the config folder.

    Returns:
    ----------
    config: dict
        The configuration dictionary containing the settings.
    """
    # Variables
    config = {}
    rospy.loginfo("Loading configuration values ...")

    try:
        # Loading configuration values
        config = rospy.get_param("~configs")

        # Check if the config is empty
        if not config:
            rospy.logerr("[Error] Configuration file is empty! Exiting ...")
            return None

        # Return the config
        rospy.loginfo("Configuration values loaded successfully!\n")
        return config
    except:
        rospy.logerr("[Error] no configuration file found! Exiting ...")
        return None
