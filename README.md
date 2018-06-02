Robot arm control tutorial with ros-control, moveit, etc.

arm_controllers is general robot arm controller.
Elfin is 6-dof manipulator. elfin_description, elfin_gazebo is forked from [3]. elfin_control and elfin_launch are added. elfin_control has control launch file and gain yaml file.

## Introduction
Implemented various control algorithm on 6-dof Elfin manipulator simulation using ros-control frameworks.

## How to run 
### Prerequisite
Install gazebo-ros-pkgs and gazebo-ros-control

    $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

Install effort-controllers to use torque-control interface

    $ sudo apt-get install ros-kinetic-effort-controllers

### Download and build 

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/modulabs/arm-tutorial.git
    $ cd ~/catkin_ws/
    $ catkin_make

### Run
Depending on the controller you want to run, use suitable launch file.
If you want to use motion controller in joint space, then you may choose this controllers as follows:

    $ roslaunch elfin_launch elfin_gravity_comp_control.launch
    or
    $ roslaunch elfin_launch elfin_time_delay_control.launch
    or
    $ roslaunch elfin_launch elfin_computed_torque_control.launch

If you want to use motion controller in task space, then you may choose this controllers as follows:

    $ roslaunch elfin_launch elfin_computed_torque_control_clik.launch

If you want to use motion and force controller in task space, then you may choose this controllers as follows:

    $ roslaunch elfin_launch elfin_adaptive_variable_impedance_control.launch 

## Reference
1. [ros-control](http://wiki.ros.org/ros_control)
2. [Write a new ros-controller](https://github.com/ros-controls/ros_control/wiki/controller_interface)
3. [Elfin manipulator](http://wiki.ros.org/Robots/Elfin)