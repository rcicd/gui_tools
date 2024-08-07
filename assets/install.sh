#!/bin/bash

source /opt/ros/iron/setup.bash

rosdep init
rosdep update
rosdep install -y --from-path ./src --rosdistro=iron

colcon build

