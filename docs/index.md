---
layout: default
title: Home
nav_order: 1
description: "Tool for programming robot behaviors with automata to generate Robotics Operating System Node."
permalink: /
---

# VisualStates
{: .fs-9 }

It represents visually the robot's behavior as a graph on a canvas composed of **states** (nodes,vertices) 
and **transitions** (edges, links). When the automaton is in a certain state it will pass from one state
to another depending on the conditions established in the transitions. This graphical representation allows 
a higher level of abstraction for the programmer.
It combines a graphical language to specify the states and the transitions with a text language (Python)
to detail the behavior on each state or the conditions in the transitions.

VisualStates generates a ROS node which implements the automata and shows a GUI at runtime with the active 
state every time, for debugging.

{: .fs-6 .fw-300 }

[Get started now](#getting-started){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 } [View it on GitHub](https://github.com/jderobot/VisualStates){: .btn .fs-5 .mb-4 .mb-md-0 }

---

## Getting started

### Installation from Ubuntu ROS Package
If you have already installed ROS on your system, you can install visualstates with the following command.
```bash
sudo apt install ros-kinetic-visualstates
```

### Installation from Source

Install the following dependencies:

##### ROS Kinetic Kame
VisualStates is built as a Robotics Operating System package and currently developed using ROS Kinetic Kame. Follow the instructions given here to install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) on your desktop.
```bash
sudo apt install ros-kinetic-desktop
```

##### PyQT
The frontend depends on PyQT to interactively develop robotic visual behavior.
```bash
sudo apt install python-pyqt5
sudo apt install python-pyqt5.qsci
```

The VisualStates tool is distributed as a ROS package. You can directly clone VisualStates repository in an active catkin workspace. Follow the steps below in your ROS workspace:
```bash
cd catkin_ws/src
git clone https://github.com/JdeRobot/VisualStates.git
cd ..
catkin_make
source devel/setup.bash
rosrun visualstates main.py
```	
---

## About the project

VisualStates is &copy; 2017-2019 by [JdeRobot](http://jderobot.org/VisualStates).

### Contributing

When contributing to this repository, please first discuss the change you wish to make via issue,
email, or any other method with the owners of this repository before making a change. Read more about becoming a contributor in [our GitHub repo](https://github.com/jderobot/VisualStates).

### Code of Conduct

VisualStates is committed to fostering a welcoming community.

[View our Code of Conduct](https://github.com/jderobot/VisualStates) on our GitHub repository.
