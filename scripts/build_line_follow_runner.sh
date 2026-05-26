set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${ROOT_DIR}/TDPS-Simulator"
SIM_TEST_DIR="${SIM_ROOT}/sim_tests"
LF_COMMON_DIR="${SIM_TEST_DIR}/common"
LF_SUITE_DIR="${SIM_TEST_DIR}/line_follow_v1"
ARTIFACT_DIR="${SIM_ROOT}/artifacts/line_follow_v1"
RUNNER_BIN="${ARTIFACT_DIR}/bin/lf_autotest_runner"

if [[ -d "${ROOT_DIR}/TDPS/firmware" ]]; then
  FW_ROOT="${ROOT_DIR}/TDPS/firmware"
elif [[ -d "${FW_ROOT}" ]]; then
  FW_ROOT="${FW_ROOT}"
else
  echo "[build] cannot find firmware sources under ${ROOT_DIR}/TDPS/firmware or ${FW_ROOT}" >&2
  exit 2
fi

mkdir -p "$(dirname "${RUNNER_BIN}")"

EXTRA_DEFS=()
for name in \
  TDPS_SIM_CALIBRATION_DURATION_MS \
  TDPS_SIM_CALIBRATION_SWITCH_MS \
  TDPS_SIM_CALIBRATION_SPIN_SPEED \
  TDPS_SIM_SENSOR_ALPHA \
  TDPS_SIM_LINE_MIN_SUM \
  TDPS_SIM_EDGE_HINT \
  TDPS_SIM_KP \
  TDPS_SIM_KI \
  TDPS_SIM_KD \
  TDPS_SIM_BASE_SPEED \
  TDPS_SIM_MAX_CORRECTION \
  TDPS_SIM_RECOVER_TURN \
  TDPS_SIM_RECOVER_TIMEOUT \
  TDPS_SIM_DYNAMIC_CALIBRATION; do
  if [[ -n "${!name:-}" ]]; then
    EXTRA_DEFS+=("-D${name}=${!name}")
  fi
done

echo "[build] compiling line-follow autotest runner..."
gcc -std=c11 -Wall -Wextra -Werror \
  -DWL_STUB_USE_LF_TIME \
  -DWL_STUB_QUIET \
  ${EXTRA_DEFS[@]+"${EXTRA_DEFS[@]}"} \
  -I"${FW_ROOT}/Inc" \
  -I"${FW_ROOT}/common" \
  -I"${FW_ROOT}/platform" \
  -I"${LF_COMMON_DIR}" \
  -I"${LF_COMMON_DIR}/harness" \
  -I"${LF_SUITE_DIR}" \
  "${FW_ROOT}/Src/lf_app.c" \
  "${FW_ROOT}/Src/lf_chassis.c" \
  "${LF_SUITE_DIR}/lf_sim_config_override.c" \
  "${FW_ROOT}/Src/lf_control.c" \
  "${FW_ROOT}/Src/lf_debug_monitor.c" \
  "${FW_ROOT}/Src/lf_sensor.c" \
  "${FW_ROOT}/Src/lf_radar.c" \
  "${FW_ROOT}/Src/lf_future_hooks.c" \
  "${FW_ROOT}/Src/wireless_hooks.c" \
  "${FW_ROOT}/Src/wl_app.c" \
  "${FW_ROOT}/Src/wl_lora.c" \
  "${FW_ROOT}/Src/wl_protocol.c" \
  "${FW_ROOT}/Src/wl_config.c" \
  "${FW_ROOT}/Src/wl_platform_stub.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_core.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_scenarios.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_evaluator.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_report.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_baseline.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_cli.c" \
  "${LF_SUITE_DIR}/lf_autotest_harness.c" \
  -lm -o "${RUNNER_BIN}"

echo "${RUNNER_BIN}"
