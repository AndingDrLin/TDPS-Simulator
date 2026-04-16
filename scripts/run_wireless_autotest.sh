set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${ROOT_DIR}/TDPS-Simulator"
ARTIFACT_BIN_DIR="${SIM_ROOT}/artifacts/tests/bin"
WL_BIN="${ARTIFACT_BIN_DIR}/wl_async_autotest"
INT_BIN="${ARTIFACT_BIN_DIR}/lf_radar_lora_integration_autotest"

if [[ -d "${ROOT_DIR}/TDPS/firmware" ]]; then
  FW_ROOT="${ROOT_DIR}/TDPS/firmware"
elif [[ -d "${ROOT_DIR}/firmware" ]]; then
  FW_ROOT="${ROOT_DIR}/firmware"
else
  echo "[wireless-test] cannot find firmware sources under ${ROOT_DIR}/TDPS/firmware or ${ROOT_DIR}/firmware" >&2
  exit 2
fi

mkdir -p "${ARTIFACT_BIN_DIR}"

echo "[wireless-test] building async LoRa autotest..."
gcc -std=c11 -Wall -Wextra -Werror \
  -I"${FW_ROOT}/Inc" \
  -I"${FW_ROOT}/common" \
  -I"${FW_ROOT}/platform" \
  "${FW_ROOT}/Src/wl_config.c" \
  "${FW_ROOT}/Src/wl_lora.c" \
  "${FW_ROOT}/Src/wl_platform_stub.c" \
  "${SIM_ROOT}/sim_tests/wireless_v1/wl_async_autotest.c" \
  -o "${WL_BIN}"

echo "[wireless-test] running async LoRa autotest..."
"${WL_BIN}"

echo "[wireless-test] building radar+LoRa integration autotest..."
gcc -std=c11 -Wall -Wextra -Werror \
  -DWL_STUB_USE_LF_TIME \
  -I"${FW_ROOT}/Inc" \
  -I"${FW_ROOT}/common" \
  -I"${FW_ROOT}/platform" \
  "${FW_ROOT}/Src/lf_app.c" \
  "${FW_ROOT}/Src/lf_chassis.c" \
  "${FW_ROOT}/Src/lf_config.c" \
  "${FW_ROOT}/Src/lf_control.c" \
  "${FW_ROOT}/Src/lf_sensor.c" \
  "${FW_ROOT}/Src/lf_radar.c" \
  "${FW_ROOT}/Src/lf_future_hooks.c" \
  "${FW_ROOT}/Src/wireless_hooks.c" \
  "${FW_ROOT}/Src/wl_app.c" \
  "${FW_ROOT}/Src/wl_config.c" \
  "${FW_ROOT}/Src/wl_lora.c" \
  "${FW_ROOT}/Src/wl_protocol.c" \
  "${FW_ROOT}/Src/wl_platform_stub.c" \
  "${SIM_ROOT}/sim_tests/integration/lf_radar_lora_integration_autotest.c" \
  -o "${INT_BIN}"

echo "[wireless-test] running radar+LoRa integration autotest..."
"${INT_BIN}"
