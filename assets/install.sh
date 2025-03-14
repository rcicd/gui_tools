#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

rosdep init
rosdep update --rosdistro=$ROS_DISTRO
rosdep install -y --from-path ./src --rosdistro=$ROS_DISTRO

colcon build

