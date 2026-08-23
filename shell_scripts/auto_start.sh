#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SRC="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOCKER_RUN_SCRIPT="${WORKSPACE_SRC}/icart_mini_ros2/docker/run.sh"
DNE_SCRIPT="${WORKSPACE_SRC}/tang2dne_handler/scripts/rs485Handler.py"
MODE_FILE="${TANG_STARTUP_MODE_FILE:-${HOME}/.config/tang/startup_mode}"
ACTIVE_MODE_FILE="${HOME}/icart_ws/log/auto_start/active_mode"
STARTUP_DELAY_SEC="${TANG_STARTUP_DELAY_SEC:-5}"

export PATH="${HOME}/.local/bin:${PATH}"

if [ -f "${MODE_FILE}" ]; then
  STARTUP_MODE="$(tr -d '[:space:]' < "${MODE_FILE}")"
else
  STARTUP_MODE="TANG"
fi
STARTUP_MODE="${STARTUP_MODE^^}"

case "${STARTUP_MODE}" in
  TANG)
    if [ ! -x "${DOCKER_RUN_SCRIPT}" ]; then
      echo "Docker launcher was not found or is not executable: ${DOCKER_RUN_SCRIPT}" >&2
      exit 1
    fi
    START_COMMAND=(
      "${DOCKER_RUN_SCRIPT}"
      ros2 launch tang_bringup tang_bringup.launch.py
    )
    ;;
  DNE)
    if [ ! -f "${DNE_SCRIPT}" ]; then
      echo "DNE handler was not found: ${DNE_SCRIPT}" >&2
      exit 1
    fi
    START_COMMAND=(
      /usr/bin/python3 -u "${DNE_SCRIPT}"
      --host 192.168.212.1
      --port-odm 18080
      --port-ctl 28080
      --robot CuGoV4
    )
    ;;
  *)
    echo "Startup mode must be TANG or DNE: ${STARTUP_MODE:-<empty>}" >&2
    echo "Change it with: ${SCRIPT_DIR}/select_startup_mode.sh tang|dne" >&2
    exit 2
    ;;
esac

echo "Selected startup mode: ${STARTUP_MODE}"
printf 'Command:'
printf ' %q' "${START_COMMAND[@]}"
printf '\n'

if [ "${TANG_STARTUP_VALIDATE_ONLY:-0}" = "1" ]; then
  exit 0
fi

# ブート直後にネットワークとデバイスが安定する時間を確保する。
sleep "${STARTUP_DELAY_SEC}"

if command -v fuser >/dev/null 2>&1 && [ -e /dev/ttyUSB0 ] && \
    fuser /dev/ttyUSB0 >/dev/null 2>&1; then
  echo "/dev/ttyUSB0 is already in use; refusing to start ${STARTUP_MODE}." >&2
  exit 1
fi

if [ "${STARTUP_MODE}" = "TANG" ]; then
  if pgrep -f 'python(3)? .*rs485Handler[.]py' >/dev/null 2>&1; then
    echo "DNE handler is already running; refusing to start TANG." >&2
    exit 1
  fi

  # 共通serviceはDNEモードでも使うためDockerへ依存させず、ここで起動を待つ。
  docker_ready=false
  for _attempt in $(seq 1 60); do
    if docker info >/dev/null 2>&1; then
      docker_ready=true
      break
    fi
    sleep 1
  done
  if [ "${docker_ready}" != "true" ]; then
    echo "Docker did not become ready within 60 seconds." >&2
    exit 1
  fi
else
  if pgrep -f "ros2 launch tang_bringup tang_bringup.launch.py" >/dev/null 2>&1; then
    echo "TANG bringup is already running; refusing to start DNE." >&2
    exit 1
  fi
fi

mkdir -p "$(dirname "${ACTIVE_MODE_FILE}")"
printf '%s\n' "${STARTUP_MODE}" > "${ACTIVE_MODE_FILE}"

exec "${START_COMMAND[@]}"
