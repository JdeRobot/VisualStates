# VisualStates
{: .fs-9 }


VisualStates is a tool for programming robot behaviors with automata (Finite State Machines, even Hierarchical ones). 

<p align="center">
  <img width="600" src="https://jderobot.github.io/VisualStates/gui-states.png"></img>
</p>

| Main features                   | 
| ------------------------------- | 
| Visual design of robot behavior | 
| C++ and Python                  | 
| Hierachical                     |
| ROS friendly                    |
| optional runtime GUI            | 

## Table of Contents
1. [Installation](#installation)
2. [Manual](#manual)
3. [Examples](#examples)
4. [Development challenges](#development-challenges)


# Installation

## Installation from Ubuntu ROS package
If you have already installed ROS on your system, you can install visualstates with the following command.
```
sudo apt install ros-kinetic-visualstates
```
To run the visualstates run the following command
```
rosrun visualstates main.py
```

## Installation from Source

### Install Dependencies
```
sudo apt install ros-kinetic-desktop
sudo apt install python-pyqt5
sudo apt install python-pyqt5.qsci
```

### Install VisualStates in catkin_ws
The VisualStates tool is distributed as a ROS package. You can directly clone this repository in an active catkin workspace. After copying the repository as ROS package. You can run visualstates following these steps.
```
catkin_make
rosrun visualstates main.py
```

To run an example behavior please check the examples repository here: [VisualStates-examples](https://github.com/JdeRobot/VisualStates-examples)


# Manual

## Using VisualStates

The VisualStates GUI has four sections: an upper menu bar, a treeView, SchemaView and dialogs for code and configs.

<p align="center">
  <img width="600" src="https://jderobot.github.io/VisualStates/visualstates-gui.png">
</p>

The menu bars contains the main buttons, which will allow you to do most of the actions. This buttons are:

#### File button
* New: it allows you to start a new empty automata. It will close your actual automata if you had anyone open, and create a new empty subautomata for starting your new automata.
* Open: it allows you to open an existing automata and continue editing it.
* Save: it save the changes in the current path of the automata you are working with. If you did not saved yet, It will show up save as dialog.
* Save as: it allows you to save the automata as an xml file in the path you choose. This path will be the new path of your project, in case you press the button save.
* Quit: it exits the tool. If you have not save your change they will be lost.

#### Figures button
* State: it allows you to create new states. For this, you must click the button and then click in the Schema wherever you want to place the new state. Ater that you can edit it. Nodes are represented as blue circles.
* Transition: it allows you to create new transitions. It works similar to the state button. You click it, and then, in the Schema view, you click the first node (state) origin and then the target one. If the target node is the same that the node origin, the tool will draw an autotransition. Transitions are represented as arrows pointing to the target node.

#### Data button
* Timer: opens a window allowing you to modify the time of the loop. This means, you can vary the frequency of condition check.
* Variables: it pop up a window. You can define any variable you need in the nodes or transitions.
* Functions: you can also define your own functions, in case you need them. Functions have a global scope, both in python and in C++, while the variables you may declare has a local scope. They will only be seen in the subautomata where you are declaring them. In python, you can made them have a global scope by declaring them with "self.", as then they will be a property of the python object automata.

#### Actions button
* Libraries: it allows you to add any library you may need. You only have to put its name.
* Config file: it allow you to select the interfaces you may need in the code of states and transitions. You can first choose the type of the communication interface: JdeRobot Communication or ROS Nodpice. If you choose JdeRobot Communication you define your interfaces from given interface where you can set Server Type, Name, Topic, Proxy Name, IP, Port, Interface. If you choose ROS Node, you define package build dependencies and run dependencies under Package tab. You can choose your ROS topic interfaces from Topics tab. You can set the name of the topic, type of the topic and whether you would like to publish or subscribe to the topic.
* Generate C++ code: when clicking in this button, the tool will generate the C++ code of the automata saved CMakeLists.txt and the yml file with the interfaces that you have already defined if it is JdeRobot Communication interface. Otherwise, it will generate package.xml, CMakeLists.txt, python runtime codes and C++ code of the automata to the given catkin workspace.
* Generate python code: generate the python code of the automata and the yml file. It works in the same way as when clicking in Generate C++ code. It will leave the automata code in executable file .py.

Now, looking at the Tree view, you can see that it displays two column: the node name and the node ID.I n the image, it is only displaying the nodes of the root subautomata, but the arrow next to its ID shows that these nodes also have sons. If you click on the arrow, the Tree view will expands and it will show this children also, so you can choose until which level do you want to see. Additionaly, you can use the Tree view for navigating through different subautomatas. If you do double click in a subautomata that you are not seeing now in the Schema view, it will redraw the Schema and show it.

Finally, there is the Schema view. It allows you to add and edit states and transitions to your current subautomata for programming its behaviour. For adding new nodes, you just click in the menu on Figures/State, and then click on the Schema, and it will draw you a new node. If you make right click on it, you will have some options for editing it:

* Rename: allows you to change its current name. By default the name given is "state". All states must have a different name. You can also rename your states if you double click on the name of the state.
* Code: it will pop up a new window where you can add the code that will be executed when this state is active.
* Mark initial: will set this node as the initial node of the subautomata. The initial nodes is the first node executed when the subautomata start for the first time. They are represented with a concentric circle.
* Copy: allows you to copy the node and then paste it in other place by doing right click/paste.
* Remove: it will erase this node and its transitions.

Also, after you have created a transition, if you right click on the red handle of the transition it will also show some options for editing it:

* Rename: set a new name. Two transitions can have the same name.
* Code: Change the transition type. There are two types of transitions: temporal and conditional. If you set it as a temporal transition, then you must set a time (in milliseconds). After that time has passed, the node will stop of being active and the node pointed by this transition will be the active one. If you set the transition as conditional, then you must declare a boolean expression and the transition will take place when the expression is evaluated to true. If you choose conditional, you are going to write your conditional expression in the code editor at the bottom part of the dialog.
* Remove: it erase the transition.
* Additionally, if you want to create a subautomata son, then you must do double click on the node you want to be the father. Then the Schema will draw a new blank subautomata for adding the nodes and transitions you need, and they will appear in the Tree. For going back to the subautomata father you only have to click on the Up button, under the Tree. This button will redraw in the Scheme the current subautomata's father or will not do anything if your subautomata does not have any parent. You can also use the Tree view for going to another subautomata. If you want to erase a subautomata, you must erase all its nodes and transitions, and then go to another subautomata (its father, for example).

## Execution of the C++ created automaton

Once you have created the code and compiled it, you will find an executable and a .yml file. When running your C++ automata with the option --displaygui=true, a runtime GUI will be shown. Its aspect is like the GUI of visualStates, but it does not have a menu bar, it only have the Schema and the Tree view just for visualization purposes.

In this case, the GUI will not allow to edit any parameter, it just shows the active nodes allowing to visualize how the automata is working and if it goes from one state to another as we were expecting. In the Tree, the active node will be shown in green and the others in white, so you can see a resume of which nodes are actives in which subautomatas, all at the same time. In the Schema, there is only one subautomata drawn. Again, the active node will be coloured in green and the rest in blue. This section allow to see which node is active in the actual subautomata and if it transit to the state that it was supposed.

The navigation is similar to the navigation in the visualStates GUI. You can go to one node child by double clicking on it and return to the current subatomata's father by clicking on the button Up, or you can just make double click on the Tree over the subautomata you want to see.

## Execution of the Python created automaton

When you press on generate python code, an executable python script and the .yml file will be created. Again, if you execute the script with the argument --displaygui=true, then a GUI window will be shown. The behaviour of this GUI is the same as it was in the C++ GUI. The way of navigating using the Tree or the Schema is the same.


# Examples

* Basic example with TurtleBot robot

[![turtlebot](https://jderobot.github.io/VisualStates/example-turtlebot.png)](https://www.youtube.com/watch?v=o-SAe_qwOMc)


* Hierarchical example Prius

[![prius](https://jderobot.github.io/VisualStates/example-prius.png)](https://www.youtube.com/watch?v=1iYlJLJkESU)

# Development Challenges

## Adding full compatibility with ROS

The current state of VisualStates only supports subscription and publish for topics. 
We aim to integrate all the communication features of the ROS and also basic packages that would be useful
for behavior development. In the scope of this project the following improvements are targeted for a new
elease of VisualStates tool:

* The integration of ROS services, the behaviors will be able to call ROS services.
* The integration of ROS actionlib, the behaviors will be able to call actionlib services.
* The generating and reading smach behaviors in VisualStates and modify and generate new behaviors.

## Library of parameterized automata

Every automaton created using VisualStates can be seen as a state itself and then be integrated in a larger
automata. Therefore, the user would be able to add previously created behaviors as states. When importing
those behaviors, the user would have two options; copying the behavior on the new behavior or keeping
reference to the imported automata such that if it is changed, those changes are going to be reflected 
on the new behavior too. The idea of this project is to built and support an automata library. There will 
be a library of predefined behaviors (automata) for coping with usual tasks, so the user can just integrate
them as new states on a new automata, without writing any code. In addition, each automaton may accept 
parameters to fine tune its behavior. For example, for moving forward a drone, we'll have a state 'moveForward', 
so the user only have to import that state indicating as a parameter the speed he wants.
