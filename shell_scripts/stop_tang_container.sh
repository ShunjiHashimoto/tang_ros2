#!/usr/bin/env bash
set -Eeuo pipefail

export PATH="${HOME}/.local/bin:${PATH}"
CONTAINER_NAME="icart_mini_ros2"

if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Stopping container: ${CONTAINER_NAME}"
  docker stop --time 20 "${CONTAINER_NAME}"
else
  echo "Container ${CONTAINER_NAME} is not running."
fi
