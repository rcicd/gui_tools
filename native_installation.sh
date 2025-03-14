#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

pip3 install -r requirements.txt

rosdep init
rosdep update
rosdep install -y --from-path ./src --rosdistro=iron

colcon build

if [ ! -e ./assets/flag ]; then
    mkdir ./tmp

    mv ./intrinsic_calibration_tool.sh ./tmp
    mv ./virtual_joystick.sh ./tmp

    mv ./assets/intrinsic_calibration_tool.sh ./
    mv ./assets/virtual_joystick.sh ./

    mv ./tmp/intrinsic_calibration_tool.sh ./assets
    mv ./tmp/virtual_joystick.sh ./assets

    rm -rf ./tmp
    touch ./assets/flag
fi