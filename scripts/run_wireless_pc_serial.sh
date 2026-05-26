#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${ROOT_DIR}/TDPS-Simulator"
ARTIFACT_BIN_DIR="${SIM_ROOT}/artifacts/tests/bin"
WL_PC_BIN="${ARTIFACT_BIN_DIR}/wl_pc_serial_test"

if [[ -d "${ROOT_DIR}/TDPS/firmware" ]]; then
  FW_ROOT="${ROOT_DIR}/TDPS/firmware"
elif [[ -d "${ROOT_DIR}/firmware" ]]; then
  FW_ROOT="${ROOT_DIR}/firmware"
else
  echo "[wireless-pc-serial] cannot find firmware sources under ${ROOT_DIR}/TDPS/firmware or ${ROOT_DIR}/firmware" >&2
  exit 2
fi

mkdir -p "${ARTIFACT_BIN_DIR}"

echo "[wireless-pc-serial] building PC serial wireless test..."
gcc -std=c11 -Wall -Wextra -Werror \
  -DWL_USE_POSIX_SERIAL_PORT \
  -I"${FW_ROOT}/Inc" \
  -I"${FW_ROOT}/common" \
  -I"${FW_ROOT}/platform" \
  "${FW_ROOT}/Src/wl_config.c" \
  "${FW_ROOT}/Src/wl_protocol.c" \
  "${FW_ROOT}/Src/wl_platform_posix_serial.c" \
  "${FW_ROOT}/Src/wl_pc_serial_main.c" \
  -o "${WL_PC_BIN}"

echo "[wireless-pc-serial] running PC serial wireless test..."
"${WL_PC_BIN}" "$@"
