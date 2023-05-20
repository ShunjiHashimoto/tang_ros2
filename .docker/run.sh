#!/bin/bash

xhost +local:root

docker run --rm \
  --net=host \
  --gpus all \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:$docker/.Xauthority \
  -v $HOME/ros2_ws:/home/appuser/ros2_ws \
  -e XAUTHORITY=$HOME/.Xauthority \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -it --name "tang_ros2" tang_ros2