#! /bin/bash

source ../../devel/setup.sh

roslaunch acrobot_gazebo acrobot_world.launch&

roslaunch acrobot_control acrobot_control.launch
