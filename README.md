# ROS2 ATR Interfaces

ROS2 Packages for the ATR communication interfaces

The documentation of the project can be found here:
<https://edeanl.github.io/ATR_Interfaces>

## Description

This repository provides ros2 packages with the communication interfaces for the ATRs.

The scenario includes the following modules:

1. ATR Formations
1. Global to Discrete Pose
1. Job-based ATR Scheduler
1. Discrete to Global Pose
1. RTSP Stream
1. ATR Tracker
1. Semantic Segmentation
1. NONA Generator
1. Dynamic Obstacle Prediction
1. ATR Trajectory Generator
1. ATR Fleet Control
1. ATR Control
1. ROS2-Redis Bridge

The ros packages have been tested in Ubuntu 20.04 with ROS Foxy.

It provides three ros packages:

- atr_interfaces: With the msg and srv definitions
- atr_examples: C++ implementations of the nodes for the ATR scenario using the atr_interfaces
- atr_examples_py: Some examples in python (needs more work)

## Usage

Copy the ros packages to your ros2 workspace:

```bash
cp atr_interfaces atr_examples /home/usr/ros_workspace/src/
```

Source your ros distro:

```bash
source /opt/ros/foxy/setup.bash
```

Compile your workspace, e.g.

```bash
cd /home/usr/ros_workspace/src/
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_STANDARD=14
```

There's a launch file (python) that will run all the nodes and visualization tools. In that python file, you can see which are the used nodes for this demo.

```bash
cd /home/usr/ros_workspace/src/
source install/setup.bash
ros2 launch atr_examples atr_interfaces_test_launch.py
```

![alt text](https://github.com/edeanl/ATR_Interfaces/blob/main/docs/Figures/atr_scenario_rviz.png "ATR scenario in Rviz")

The active nodes for this demo are the following:

![alt text](https://github.com/edeanl/ATR_Interfaces/blob/main/docs/Figures/rqt_nodes.png "ATR scenario nodes")

## TODO

The documentation of the modules is not complete, for now, the links point to the same draft pdf file.

Missing modules:

- Global to Discrete Pose
- Job-based ATR Scheduler
- Discrete to Global Pose
- RTSP Stream
- ROS2-Redis Bridge
  