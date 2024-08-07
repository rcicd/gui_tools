#!/bin/bash

source /opt/ros/iron/setup.bash

pip install -r requirements.txt

#rosdep init
#rosdep update
#rosdep install --from-path ./src --rosdistro=iron

colcon build