#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_SOURCE="${SCRIPT_DIR}/startup_robot.service"
SERVICE_DESTINATION="/etc/systemd/system/startup_robot.service"
MODE_FILE="${HOME}/.config/tang/startup_mode"

if [ ! -f "${SERVICE_SOURCE}" ]; then
  echo "Service definition was not found: ${SERVICE_SOURCE}" >&2
  exit 1
fi

if [ ! -f "${MODE_FILE}" ]; then
  mkdir -p "$(dirname "${MODE_FILE}")"
  printf 'TANG\n' > "${MODE_FILE}"
fi

sudo install -m 0644 "${SERVICE_SOURCE}" "${SERVICE_DESTINATION}"
sudo systemctl daemon-reload
if systemctl list-unit-files startup_tang.service --no-legend 2>/dev/null | \
    grep -q '^startup_tang.service'; then
  sudo systemctl disable startup_tang.service
fi
sudo systemctl enable startup_robot.service

echo "Robot auto-start is enabled for the next boot."
echo "Selected mode: $(tr -d '[:space:]' < "${MODE_FILE}")"
echo "Switch mode:   ${SCRIPT_DIR}/select_startup_mode.sh tang|dne"
echo "Start it now:  sudo systemctl start startup_robot.service"
echo "Follow logs:   journalctl -u startup_robot.service -f"
