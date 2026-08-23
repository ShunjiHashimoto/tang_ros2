#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ACTIVE_MODE_FILE="${HOME}/icart_ws/log/auto_start/active_mode"

if [ -f "${ACTIVE_MODE_FILE}" ]; then
  ACTIVE_MODE="$(tr -d '[:space:]' < "${ACTIVE_MODE_FILE}")"
else
  ACTIVE_MODE=""
fi

case "${ACTIVE_MODE^^}" in
  TANG)
    "${SCRIPT_DIR}/stop_tang_container.sh"
    ;;
  DNE)
    # systemdがMainPIDへSIGTERMを送り、rs485Handlerが停止指令とcloseを行う。
    echo "Stopping DNE handler through systemd."
    ;;
  *)
    echo "No active startup mode was recorded."
    ;;
esac

rm -f "${ACTIVE_MODE_FILE}"
