Experiment control algorithm using acrobot mechanism </br>
Acrobot is double pendulum mechanism having 1 passive and 1 active joint

## Introduction
Acrobot control algorithm implemented.</br>
In swing-up phase, task space partial feedback linearization used,</br>
Near origin, lqr used,</br>
Switching between 2 phases, region of attraction algorithm have to be implemented (not yet)

## Usage
- Prerequisite</br>
 1. Install gazebo-ros-pkgs and gazebo-ros-control (might already installed)</br>
```sh
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
``` 
  2. Install effort-controllers for torque-control</br>
```sh
sudo apt-get install ros-kinetic-effort-controllers
```
- Download and build 
```sh
cd ~/catkin_ws/
catkin_make
```

- Download acrobot_gazebo, acrobot_description, acrobot_control package in this repository at ~/catkin_ws/src/

- If you want to use python code, authorize acrobot_control/scripts/acrobot_control.py
```sh
chmod +x acrobot_control.py
```

- Launch acrobot_gazebo
```sh
roslaunch acrobot_gazebo acrobot_world.launch
```

- Launch acrobot_control
```sh
roslaunch acrobot_control acrobot_control.launch
```
or if you want to use python code
```sh
roslaunch acrobot_control acrobot_control_python.launch
```


## To do
1. Region of attraction 

## Reference
1. tedrake lecture
2. tedrake paper
3. spong swing up
4. sos
5. <http://gazebosim.org/tutorials?cat=connect_ros>
6. <https://github.com/JoshMarino/gazebo_and_ros_control>
