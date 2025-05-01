#!/usr/bin/env bash

export PATH="$HOME/.local/bin:$PATH"
CONTAINER_NAME="icart_mini_ros2"

IS_RUNNING=$(docker ps --format '{{.Names}}' | grep -w "$CONTAINER_NAME" | wc -l)

if [ "$IS_RUNNING" -eq 1 ]; then
    echo "Stopping ROS 2 node inside container..."
    docker exec -i $CONTAINER_NAME pkill -SIGINT ros
    echo "Successfully sent SIGINT to ros2 node."

    echo "Stopping container..."
    docker stop $CONTAINER_NAME
    echo "Successfully stopped container."
else
    echo "Container $CONTAINER_NAME is not running."
fi
