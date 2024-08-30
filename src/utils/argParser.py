import argparse


def argParser(mode: str):
    # Variables
    validModes = ["offimg", "offvid", "usb", "ids", "rs"]
    # Create an argument parser
    parser = argparse.ArgumentParser()
    # Add arguments to override config values
    parser.add_argument(
        '--mode', type=str, help="Override runner mode (rs, ids, usb, offvid, offimg)")
    # New mode
    newMode = parser.parse_args().mode
    # Check if the mode is valid
    if newMode and newMode in validModes and newMode != mode:
        print(
            f'[Info] a new mode "{newMode}" is set using arguments, which is different from "{mode}" in the config file ...')
        mode = newMode
    # Check if the mode is valid
    if newMode and newMode not in validModes:
        print(
            f'[Warn] skipping the mode "{newMode}" set using arguments due to invalidity. Reading mode from the config file ...')
    # Return the parsed arguments
    return mode
