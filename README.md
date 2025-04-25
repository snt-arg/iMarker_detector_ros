# iMarker Detector ROS (with Rviz)

![Demo](docs/demo.gif "Demo")

This repository contains the **ROS-1** version of iMarker and CSR-based object detector, titled `imarker_detector_ros`, mainly designed for **robotics** tasks. It provides a wrapper over [iMarker_sensors](https://github.com/snt-arg/iMarker_sensors) and [iMarker_algorithms](https://github.com/snt-arg/iMarker_algorithms) and uses an `Rviz` user interface for monitoring the results.

⚠️ In case you do not want to use the ROS-enabled version, you can also clone the [GUI-enabled standalone version](https://github.com/snt-arg/iMarker_algorithms_standalone).

## 🛠️ Getting Started

### I. Cloning the Repository

Create a new ROS workspace and clone the repo in the `src` folder. Note that the current repository has some submodules that is highly dependent on them (added using the command `git submodule add git@[repo].git src/[name]`).

Hence, when cloning the repository make sure to include `--recurse-submodules` after `git clone` such that the submodules are added as well, as below:

```
git clone --recurse-submodules git@github.com:snt-arg/imarker_detector_ros.git
```

You can also get the latest changes of each submodule individually using the command `git pull --recurse-submodules`.

💡 **[note]** In case you do not have SSH access, you can just download the code of [this library](https://github.com/snt-arg/iMarker_algorithms_standalone), and clone the [detector sensors](https://github.com/snt-arg/iMarker_sensors) inside `src/iMarker_sensors`, and [detector algorithms repo](https://github.com/snt-arg/iMarker_algorithms) inside `src/iMarker_algorithms` paths.

After cloning the repository, you can add a command like `alias sourcecsr='source ~/workspace/ros/imarker_detector_ros_ws/devel/setup.bash'` in your `.bashrc` file.

### II. Installation

After cloning the repository, you need to install the required dependencies. The Python version used while developing the framework is `3.10.4`. It is highly recommended to create a Python virtual environment using `python -m venv .venv`, activate it using `source .venv/bin/activate`, and then install the required dependencies in the `requirements.txt` using the below command:

```
pip install -r requirements.txt
```

You can also install the cloned submodules and define dependencies and other distribution-related configurations using the provided `setup.py` file in the root directory of each file. Hence, follow the below steps:

- Go to `src/iMarker_sensors` and run `pip install -e .`,
- Go to `src/iMarker_algorithms` and run `pip install -e .`,
- Go to the **root directory** and run `pip install -e .` to install the package and its dependencies.

### III. Build Catkin Package

Finally, when everything has been installed, you can run `catkin build` to build the files.

## 🚀 Running the Code

### I. Set Configurations

The first step is to modify the configuration file. For a complete list of configurations you can take a look at [config.yaml](/config/config.yaml) or read the detailed descriptions [here](/config/README.md).

⚠️ **[hint]** you can also find specified configuration files in the same folder, listed below.

| Launcher                                           | Description                                       |
| -------------------------------------------------- | ------------------------------------------------- |
| [`cfg_off.yaml`](/config/cfg_off.yaml)             | configurations for the offline (rosbag file) mode |
| [`cfg_dual_usb.yaml`](/config/cfg_single_rs.yaml)  | configurations for ELP USB camera mode            |
| [`cfg_dual_ids.yaml`](/config/cfg_dual_ids.yaml)   | configurations for iDS camera mode                |
| [`cfg_single_rs.yaml`](/config/cfg_single_rs.yaml) | configurations for RealSense camera mode          |
| [`config.yaml`](/config/config.yaml)               | configurations for all cameras (combined)         |

## II. Run the Desired Mode

When everything is ready, you can source the workspace (running `sourcecsr` as described before) and run one of the launch files listed below:

| Launcher                          | Description                                                 |
| --------------------------------- | ----------------------------------------------------------- |
| `iMarker_detector_usb.launch`     | runs the double-vision USB camera version of the code       |
| `iMarker_detector_offline.launch` | runs the double-vision iDS cameras version of the code      |
| `iMarker_detector_rs.launch`      | runs the single-vision RealSense camera version of the code |
| `iMarker_detector_ids.launch`     | runs the single-vision offline version of the code          |

You can also configure the parameters in the launch file, including the below list:

- `show_rviz`: runs an `Rviz` node to show the framework inputs/outputs when running (default: true)

For instance, the below command runs the RealSense version of CSR detector while not running Rviz:

```bash
# Source ROS
source /opt/ros/noetic/setup.bash

# Source the workspace
source ~/[workspace]/devel/setup.bash

# Activate the .venv
source ~/[workspace]/src/imarker_detector_ros/.venv/bin/activate

# Launch the desired launch file
roslaunch imarker_detector_ros iMarker_detector_[x].launch [show_rviz:=false]
```

## 🤖 ROS Topics and Params

### Subscribed Topics

| Topic                     | Description                                                                   |
| ------------------------- | ----------------------------------------------------------------------------- |
| `/camera/color/image_raw` | for offline mode, modifiable in `sensor`/`offline`/`rosbag`/`raw_image_topic` |

### Published Topics

| Topic               | Description                                                  |
| ------------------- | ------------------------------------------------------------ |
| `/raw_img`          | publishes the main camera output of the mono-vision setup    |
| `/raw_img_left`     | publishes the left camera output of the double-vision setup  |
| `/raw_img_right`    | publishes the right camera output of the double-vision setup |
| `/mask_img`         | publishes the genetated mask to detect CSRs/iMarkers         |
| `/mask_applied_img` | publishes the genetated mask applied to the raw image        |
| `/marker_img`       | publishes the detected iMarker information                   |
| `/rs_cam_params`    | publishes the camera parameters of RealSense                 |

## 🔩 ArUco Marker Recognition

By default, the ArUco marker recognition library is built-in in all setups, processing `/mask_img` and publishing to `/marker_img`.

However, you can also run `aruco_ros` library (ROS-1 branch) [link](https://github.com/pal-robotics/aruco_ros) separately and feed it with `/mask_img` and `/rs_cam_params` topic. For doing this, you should follow below steps:

- Create a separate `launch` file for `aruco_ros` library. It should remap `/mask_img` and `/rs_camera_params` of the repository with `/image` and `/camera_info` topics of `aruco_ros`, respectively. A sample can be found [here](docs/aruco_ros_imarker.launch).
- Run the program using `roslaunch imarker_detector_ros iMarker_detector_rs.launch`

## 📝 TODOs

You can find the list of future improvements and TODO list of the framework:

- Publish the poses of the detected marker
- Add the ability to capture images, similar to the standalone version
- Publish the camera parameters of other setups to be used in `aruco_ros`
