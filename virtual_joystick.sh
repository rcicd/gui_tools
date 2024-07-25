#!/bin/bash


if [ -z "$1" ]
then
  echo "Need robot name"
  exit 1
fi

docker run --entrypoint /app/virtual_joystick.sh \
        -e DISPLAY=$DISPLAY \
        --network=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw   \
        -it --rm spgc/gui_tools:latest $1