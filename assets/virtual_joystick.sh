#!/bin/bash


if [ -z "$1" ]
then
  echo "Need robot name"
  exit 1
fi
source /opt/ros/$ROS_DISTRO/setup.bash
source ./install/setup.bash

ros2 launch virtual_joystick virtual_joystick.launch vehicle:="$1"
