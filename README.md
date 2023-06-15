# CSR Detector ROS (with Rviz)

This repository contains the ROS version of CSR-based object detector which provides a wrapper over [csr_sensors](https://github.com/snt-arg/csr_sensors) and [csr_detector](https://github.com/snt-arg/csr_detector) and provides an Rviz user interface for monitoring the results. Accordingly, the application is dependant to the repositories below:

## 📚 Preparation

### I. Cloning

Create a new workspace and clone the repo in its `src` folder. This repo has some submodules which have been added using the command `git submodule add git@[repo].git src/[name]`. When cloning the repository include `--recurse-submodules` after `git clone` such that the submodules are added as well. Accordingly, you can use the command below:

```
git clone --recurse-submodules git@github.com:snt-arg/csr_detector_ros.git
```

You can also get the latest changes of each submodule individually using the command `git pull --recurse-submodules`. After cloning the repository, you can add a command like `alias sourcecsr='source ~/workspace/ros/csr_detector_ros_ws/devel/setup.sh'` in your `.bashrc` file.

### II. Installing Libraries

Install the required Python libraries for running this program using the command below:

```
pip install numpy opencv-python
```

### III. Installing Submodule Packages

The next step is to intall the cloned submodules and define dependencies and other distribution-related configurations using the provided `setup.py` file in the root directory. Then, run `pip install -e .` in the **root directory** to install the package and its dependencies. You can also run the same command in the submodules directories to install them.

## 🤖 ROS Topics and Params

### Subscribed topics

| Topic | Description |
| ------------ | ------------ |
| `/sample` | sample |

### Published topics

| Topic | Description |
| ------------ | ------------ |
| `/sample` | sample |

### Params

| Param | Description |
| ------------ | ------------ |
| `/sample` | sample |