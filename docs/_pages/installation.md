---
layout: splash
title: Installation
permalink: /installation
---

## Installation from Ubuntu ROS package
Coming Soon.

## Installation from Source

### Install Dependencies
```
sudo apt install ros-melodic-desktop
sudo apt install python-pyqt5
sudo apt install python-pyqt5.qsci
```

### Install VisualStates in catkin_ws
The VisualStates tool is distributed as a ROS package. You can directly clone this repository in an active catkin workspace. After copying the repository as ROS package, you need to setup the sources and can run visualstates following these steps.
```
catkin_make
rosrun visualstates main.py
```

To run an example behavior please check the examples repository here: [VisualStates-examples](https://github.com/JdeRobot/VisualStates-examples)

