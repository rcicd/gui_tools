#!/bin/bash


if [ -z "$1" ]
then
  echo "Need robot name"
  exit 1
fi
source /opt/ros/iron/setup.bash
source ./install/setup.bash

ros2 launch intrinsic_calibration_tool intrinsic_calibration_tool.launch vehicle:="$1"
