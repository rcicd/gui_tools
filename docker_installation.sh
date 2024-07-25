#!/bin/bash

if [ -e ./assets/flag ]; then
    mkdir ./tmp

    mv ./intrinsic_calibration_tool.sh ./tmp
    mv ./virtual_joystick.sh ./tmp

    mv ./assets/intrinsic_calibration_tool.sh ./
    mv ./assets/virtual_joystick.sh ./

    mv ./tmp/intrinsic_calibration_tool.sh ./assets
    mv ./tmp/virtual_joystick.sh ./assets

    rm -rf ./tmp
    rm ./assets/flag
fi

docker build --tag spgc/gui_tools:latest .
