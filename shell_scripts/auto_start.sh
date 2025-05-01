#!/usr/bin/env bash

sleep 5
export PATH="$HOME/.local/bin:$PATH"
cd $HOME/icart_ws/src/icart_mini_ros2/docker && ./run.sh ros2 launch icart_mini_bringup icart_mini_bringup.launch.py
