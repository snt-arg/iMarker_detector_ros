import rospy


def readConfig():
    """
    Read the configuration file and return the configuration values.
    """
    # Loading configuration values
    try:
        rospy.loginfo("Loading configuration values ...")
        config = rospy.get_param("~configs")
        rospy.loginfo("Configuration values loaded successfully!\n")
        return config
    except:
        rospy.logerr("[Error] no configuration file found! Exiting ...")
        return None
