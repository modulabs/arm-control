Robot arm control tutorial with ros-control, moveit, etc.

arm_controllers is general robot arm controller.
Elfin is 6-dof manipulator. elfin_description, elfin_gazebo is forked from [3], elfin_launch are added. elfin_control has controller and gain in yaml file.

## Introduction
Implemented various control algorithm on 6-dof Elfin manipulator simulation using ros-control frameworks.

## How to run 
### Prerequisite
Install gazebo-ros-pkgs and gazebo-ros-control

    $ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

Install ros-controllers

    $ sudo apt-get install ros-noetic-ros-controllers

### Download and build 

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/modulabs/arm-control.git
    $ cd ~/catkin_ws/
    $ catkin_make

### Run
Depending on the controller you want to run, use suitable launch file.
If you want to use motion controller in joint space, then you may choose this controllers as follows:

    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=gravity_comp_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=time_delay_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=computed_torque_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=computed_torque_clik_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=passivity_controller

If you want to use motion controller in task space, then you may choose this controllers as follows:

    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=computed_torque_clik_controller

If you want to use motion and force controller in task space, then you may choose this controllers as follows:

    $ roslaunch elfin_gazebo elfin3_experiment1_world.launch controller:=adaptive_impedance_controller
    or
    $ roslaunch elfin_gazebo elfin3_experiment2_world.launch controller:=adaptive_impedance_controller

If you want to plot data in rqt graph, use rqt_plot.launch file. Customize perspective files to plot data you need.

    $ roslaunch rqt_plot.launch controller:=gravity_comp_controller

## Reference
1. [ros-control](http://wiki.ros.org/ros_control)
2. [Write a new ros-controller](https://github.com/ros-controls/ros_control/wiki/controller_interface)
3. [Elfin manipulator](http://wiki.ros.org/Robots/Elfin)
4. [Tomei, A Simple PD Controller for Robots with Elastic Joints](https://ieeexplore.ieee.org/document/90238)
5. [Slotine, On the Adaptive Control of Robot Manipulators](https://journals.sagepub.com/doi/10.1177/027836498700600303)
6. [Duan Jinjun, Adaptive variable impedance control](https://dl.acm.org/doi/10.1016/j.robot.2018.01.009)
