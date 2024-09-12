# iMarker Detector ROS (with Rviz)

![Demo](docs/demo.gif "Demo")

This repository contains the **ROS-1** version of iMarker and CSR-based object detector, titled `csr_detector_ros`, mainly designed for **robotics** tasks. It provides a wrapper over [csr_sensors](https://github.com/snt-arg/csr_sensors) and [csr_detector](https://github.com/snt-arg/csr_detector) and uses an `Rviz` user interface for monitoring the results.

⚠️ In case you do not want to use the ROS-enabled version, you can also clone the [GUI-enabled standalone version](https://github.com/snt-arg/csr_detector_standalone).

## 🛠️ Getting Started

### I. Cloning the Repository

Create a new ROS workspace and clone the repo in the `src` folder. Note that the current repository has some submodules that is highly dependent on them (added using the command `git submodule add git@[repo].git src/[name]`).

Hence, when cloning the repository make sure to include `--recurse-submodules` after `git clone` such that the submodules are added as well, as below:

```
git clone --recurse-submodules git@github.com:snt-arg/csr_detector_ros.git
```

You can also get the latest changes of each submodule individually using the command `git pull --recurse-submodules`.

💡 **[note]** In case you do not have SSH access, you can just download the code of [this library](https://github.com/snt-arg/csr_detector_standalone), and clone the [detector sensors](https://github.com/snt-arg/csr_sensors) inside `src/csr_sensors`, and [detector algorithms repo](https://github.com/snt-arg/csr_detector) inside `src/csr_detector` paths.

After cloning the repository, you can add a command like `alias sourcecsr='source ~/workspace/ros/csr_detector_ros_ws/devel/setup.bash'` in your `.bashrc` file.

### II. Installation

After cloning the repository, you need to install the required dependencies. The Python version used while developing the framework is `3.10.4`. It is highly recommended to create a Python virtual environment using `python -m venv .venv`, activate it using `source .venv/bin/activate`, and then install the required dependencies in the `requirements.txt` using the below command:

```
pip install -r requirements.txt
```

You can also install the cloned submodules and define dependencies and other distribution-related configurations using the provided `setup.py` file in the root directory of each file. Hence, follow the below steps:

- Go to `src/csr_sensors` and run `pip install -e .`,
- Go to `src/csr_detector` and run `pip install -e .`,
- Go to the **root directory** and run `pip install -e .` to install the package and its dependencies.

### III. Build Catkin Package

Finally, when everything has been installed, you can run `catkin build` to build the files.

## 🚀 Running the Code

### I. Set Configurations

The first step is to modify the configuration file. For a complete list of configurations you can take a look at [config.yaml](/config/config.yaml) or read the detailed descriptions [here](/config/README.md).

⚠️ **[hint]** you can also find specified configuration files in the same folder, listed below.

| Launcher          | Description                                       |
| ----------------- | ------------------------------------------------- |
| `config_off.yaml` | configurations for the offline (rosbag file) mode |
| `config_usb.yaml` | configurations for ELP USB camera mode            |
| `config_ids.yaml` | configurations for iDS camera mode                |
| `config_rs.yaml`  | configurations for RealSense camera mode          |

## II. Run the Desired Mode

When everything is ready, you can source the workspace (running `sourcecsr` as described before) and run one of the nodes listed below:

| Launcher                      | Description                                   |
| ----------------------------- | --------------------------------------------- |
| `csr_detector_usb.launch`     | Runs the USB cameras version of the code      |
| `csr_detector_ids.launch`     | Runs the iDS cameras version of the code      |
| `csr_detector_rs.launch`      | Runs the RealSense camera version of the code |
| `csr_detector_usbMono.launch` | Runs the mono USB camera version of the code  |

There are also some arguments that you can configure based on your scenario:

- `show_rviz`: Runs an Rviz node when running the main node (default: true)

For instance, the below command runs the RealSense version of CSR detector while not running Rviz:

```
roslaunch csr_detector_ros csr_detector_rs.launch show_rviz:=false
```

## 🤖 ROS Topics and Params

### Subscribed topics

| Topic | Description |
| ----- | ----------- |
| `-`   | N/A         |

### Published topics

| Topic           | Description                                         |
| --------------- | --------------------------------------------------- |
| `/left_camera`  | Publishes the left camera of a two-camera setup     |
| `/right_camera` | Publishes the right camera of a two-camera setup    |
| `/main_camera`  | Publishes the main camera of a mono-camera setup    |
| `/result_frame` | Publishes the resulting frame after processing      |
| `/result_mask`  | Publishes the resulting frame mask after processing |

### Params

| Param | Description |
| ----- | ----------- |
| `-`   | N/A         |

## 🔩 ArUco Marker Recognition

In order to recognize markers, you need to run `aruco_ros` library [link](https://github.com/pal-robotics/aruco_ros) separately and feed it with `/result_mask` and `/rs_camera_params` topic. For doing this, you should follow below steps:

- Create a separate `launch` file for `aruco_ros` library. It should remap `/result_mask` and `/rs_camera_params` of the repository with `/image` and `/camera_info` topics of `aruco_ros`, respectively. A sample can be found [here](docs/aruco_ros_csr_marker.launch).
- Run the program using `roslaunch csr_detector_ros csr_detector_rs.launch`

⚠️ The current version of the code works for Mono-camera RealSense library only. Other sensors will have this ability later.
