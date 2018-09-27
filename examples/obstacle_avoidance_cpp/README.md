# obstacle_avoidance Behavior
This is an example behavior where a turtlebot robot moves forward until it gets very close to one of the obstacles in front of it. It detects the obstacle using laser sensor.

## Steps to run the example
### Dependencies
We assume that you already installed ROS and Gazebo on Ubuntu 16.04 system to be able to test the behaviors. However, if you did not install yet, you can do so following these pages: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)  [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

In addition to ROS and Gazebo, you also need to install kobuki to be able to simulate kobuki/turtlebot robot.
```
sudo apt install ros-kinetic-kobuki-gazebo
sudo apt install ros-kinetic-kobuki-gazebo-plugins
```
We also need worlds from JdeRobot repositories. To be able to install jderobot gazebo assets. Please follow the steps here: https://jderobot.org/Installation
We add required commands here for completeness
```
sudo sh -c 'cat<<EOF>/etc/apt/sources.list.d/jderobot.list
# for ubuntu 16.04 LTS (64 bit)
deb [arch=amd64] http://jderobot.org/apt xenial main
EOF'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 24E521A4
sudo apt update
sudo apt install jderobot-gazebo-assets
```
To be able to complete the JdeRobot Gazebo assets installation we need to source the JdeRobot installation script
```
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
```
***This is important that you need to run your gazebo simulation on the console that you have sourced your JdeRobot installation otherwise Gazebo simulator cannot find models and worlds of JdeRobot***

### ROS package generation
First we must generate the ros package of the behavior using the **visualstates**.
```
rosrun visualstates main.py <path_to_visualstates>/examples/obstacle_avoidance_cpp/obstacle_avoidance_cpp.xml
```
Using `File -> Save As` save the behavior in an empty directory that is also in an active `catkin_workspace`. Since code generation will create required files to make the directory a ROS package, you should have different directory for every new behavior. Now, we can generate ROS package using `Actions -> Generate C++` menu.

Navigate to your `catkin_workspace` and run `catkin_make`. As an output of `catkin_make` you will see the `obstacle_avoidance_cpp` listed as a ROS package.
```
cd catkin_ws
catkin_make
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
rosrun obstacle_avoidance_cpp obstacle_avoidance_cpp --displaygui=true
```
