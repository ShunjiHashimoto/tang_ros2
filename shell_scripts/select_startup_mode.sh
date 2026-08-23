#!/usr/bin/env bash
set -Eeuo pipefail

MODE_FILE="${TANG_STARTUP_MODE_FILE:-${HOME}/.config/tang/startup_mode}"

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 tang|dne" >&2
  exit 2
fi

STARTUP_MODE="${1^^}"
if [ "${STARTUP_MODE}" != "TANG" ] && [ "${STARTUP_MODE}" != "DNE" ]; then
  echo "Startup mode must be tang or dne: $1" >&2
  exit 2
fi

mkdir -p "$(dirname "${MODE_FILE}")"
TEMP_MODE_FILE="$(mktemp "${MODE_FILE}.XXXXXX")"
trap 'rm -f "${TEMP_MODE_FILE}"' EXIT
printf '%s\n' "${STARTUP_MODE}" > "${TEMP_MODE_FILE}"
mv "${TEMP_MODE_FILE}" "${MODE_FILE}"
trap - EXIT

echo "Startup mode: ${STARTUP_MODE}"
echo "The selected mode will be used at the next boot."
echo "Apply it now: sudo systemctl restart startup_robot.service"
