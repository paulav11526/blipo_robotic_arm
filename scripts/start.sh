#!/bin/bash

source /opt/ros/noetic/setup.bash
soource ~/jetcobot_ws/devel/setup.bash

# start roscore
roscore &
sleep 5

# Launch blipo.launch
roslaunch jetcobot_moveit blipo.launch