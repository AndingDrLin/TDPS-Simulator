set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${ROOT_DIR}/TDPS-Simulator"
ARTIFACT_BIN_DIR="${SIM_ROOT}/artifacts/tests/bin"
RADAR_BIN="${ARTIFACT_BIN_DIR}/lf_radar_autotest"

if [[ -d "${ROOT_DIR}/TDPS/firmware" ]]; then
  FW_ROOT="${ROOT_DIR}/TDPS/firmware"
elif [[ -d "${ROOT_DIR}/firmware" ]]; then
  FW_ROOT="${ROOT_DIR}/firmware"
else
  echo "[radar-test] cannot find firmware sources under ${ROOT_DIR}/TDPS/firmware or ${ROOT_DIR}/firmware" >&2
  exit 2
fi

mkdir -p "${ARTIFACT_BIN_DIR}"

echo "[radar-test] building radar parser autotest..."
gcc -std=c11 -Wall -Wextra -Werror \
  -I"${FW_ROOT}/Inc" \
  -I"${FW_ROOT}/common" \
  -I"${FW_ROOT}/platform" \
  "${FW_ROOT}/Src/lf_config.c" \
  "${FW_ROOT}/Src/lf_radar.c" \
  "${SIM_ROOT}/sim_tests/line_follow_v1/lf_radar_autotest.c" \
  -o "${RADAR_BIN}"

echo "[radar-test] running..."
"${RADAR_BIN}"
