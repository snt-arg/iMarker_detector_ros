def thresholdParser(thresholdMethod: str):
    """
    Returns three different binary values based on the given thresholding method

    Parameters
    -------
    thresholdMethod: str
        The input thresholding method

    Returns
    -------
    isThreshOts: bool
        Is True if the thresholding method is Otsu
    isThreshBin: bool
        Is True if the thresholding method is Binary
    isThreshBoth: bool
        Is True if the thresholding method is Otsu+Binary
    """
    isThreshOts = True if (thresholdMethod == 'otsu') else False
    isThreshBoth = True if (thresholdMethod == 'both') else False
    isThreshBin = True if (thresholdMethod == 'binary') else False
    return isThreshOts, isThreshBoth, isThreshBin


def channelParser(channel: str):
    """
    Returns four different binary values based on the given frame channel

    Parameters
    -------
    channel: str
        The input frame channel

    Returns
    -------
    isRChannel: bool
        Is True if the channel is Red
    isGChannel: bool
        Is True if the channel is Green
    isBChannel: bool
        Is True if the channel is Blue
    isAllChannels: bool
        Is True if the channel is RGB
    """
    isRChannel = True if (channel == 'r') else False
    isGChannel = True if (channel == 'g') else False
    isBChannel = True if (channel == 'b') else False
    isAllChannels = True if (channel == 'all') else False
    return isRChannel, isGChannel, isBChannel, isAllChannels
