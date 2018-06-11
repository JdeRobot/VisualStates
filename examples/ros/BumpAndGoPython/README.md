BumpAndGo Automata

Steps to reproduce the Automata

First we must generate the automata code, so we go to make a directory to save the automata:
```
mkdir BumpAndGoPython && cd BumpAndGoPython
cp /opt/VisualStates/share/examples/ros/BumpAndGoPython/BumpAndGoPython.xml .
```

Open VisualStates tool, and open the BumpAndGo.xml behaviour. Save it and click on Actions/Generate Python Code option and the tool would generate the necessary files automatically as  a ROS Package. Once the ROS package is generated we copy the package into our catkin workspace and compile the package.

```
cp -r BumpAndGoPython catkin_ws/src/
cd catkin_ws
catkin_make
source devel/setup.bash
```

Start the roscore
```
roscore
```
Start the Gazebo simulator, using the gazebo_ros package
```
rosrun gazebo_ros gazebo kobuki-simple-ros.world
```
Run our generated package
```
rosrun BumpAndGoPython BumpAndGoPython.py --displaygui=true
```

You would see the Kobuki robot performing the BumpAndGo behaviour.


