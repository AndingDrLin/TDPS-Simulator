set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${ROOT_DIR}"

bash "${SCRIPT_DIR}/line_follow_cli.sh" quick 15 0.01 0.12
bash "${SCRIPT_DIR}/run_radar_autotest.sh"
bash "${SCRIPT_DIR}/run_wireless_autotest.sh"

echo "[system-autotest] all checks passed"
