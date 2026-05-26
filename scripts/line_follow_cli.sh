set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER_BIN="$(bash "${SCRIPT_DIR}/build_line_follow_runner.sh" | tail -n 1)"

if [[ $# -eq 0 ]]; then
  "${RUNNER_BIN}" --help
  exit 2
fi

set +e
"${RUNNER_BIN}" "$@"
EXIT_CODE=$?
set -e

exit ${EXIT_CODE}
