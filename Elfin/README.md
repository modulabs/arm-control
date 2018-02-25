Time delayed control algorithm on 6-dof Elfin manipulator

## Introduction
Implemented time delayed control algorithm on 6-dof Elfin manipulator simulation using ros-control frameworks.

## How to run 
### Prerequisite
Install gazebo-ros-pkgs and gazebo-ros-control

    $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

Install effort-controllers to use torque-control interface

    $ sudo apt-get install ros-kinetic-effort-controllers

### Download and build 

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/modulabs/gdyn-arm-tutorial.git
    $ cd ~/catkin_ws/
    $ catkin_make

### Run

    $ roslaunch acrobot_gazebo acrobot_world.launch

or simply

    $ ./run.sh

## Reference
1. [ros-control] (http://wiki.ros.org/ros_control)
2. [Write a new ros-controller] (https://github.com/ros-controls/ros_control/wiki/controller_interface)
3. [Elfin manipulator] (http://wiki.ros.org/Robots/Elfin)
