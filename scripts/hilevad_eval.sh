#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

die() {
  echo "error: $*" >&2
  exit 1
}

add_pythonpath() {
  local path="$1"
  if [ -e "$path" ]; then
    export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}${path}"
  fi
}

CARLA_HOME="${CARLA_HOME:-${CARLA_PACKAGE_ROOT:-${CARLA_ROOT:-}}}"
[ -n "$CARLA_HOME" ] || die "set CARLA_ROOT to your CARLA package path"

if [ -x "${CARLA_HOME}/CarlaUE4.sh" ]; then
  CARLA_LAUNCH_ROOT="$CARLA_HOME"
  CARLA_API_ROOT="$CARLA_HOME"
elif [ -x "${CARLA_HOME}/Linux/CarlaUE4.sh" ]; then
  CARLA_LAUNCH_ROOT="${CARLA_HOME}/Linux"
  CARLA_API_ROOT="$CARLA_HOME"
else
  die "cannot find CarlaUE4.sh under CARLA_ROOT=${CARLA_HOME}"
fi

export CARLA_ROOT="$CARLA_LAUNCH_ROOT"
export SCENARIO_RUNNER_ROOT="${REPO_ROOT}/scenario_runner"
export LEADERBOARD_ROOT="${REPO_ROOT}/leaderboard"
export CHALLENGE_TRACK_CODENAME="${CHALLENGE_TRACK_CODENAME:-SENSORS}"

add_pythonpath "${CARLA_API_ROOT}/PythonAPI"
add_pythonpath "${CARLA_API_ROOT}/PythonAPI/carla"
for egg in "${CARLA_API_ROOT}"/PythonAPI/carla/dist/carla-*.egg; do
  [ -e "$egg" ] && add_pythonpath "$egg"
done
add_pythonpath "${REPO_ROOT}/leaderboard"
add_pythonpath "${REPO_ROOT}/leaderboard/team_code"
add_pythonpath "${REPO_ROOT}/scenario_runner"

ROUTES="${ROUTES:-${REPO_ROOT}/leaderboard/data/HLADs.xml}"
ROUTES_SUBSET="${ROUTES_SUBSET:-}"
TEAM_AGENT="${TEAM_AGENT:-}"
TEAM_CONFIG="${TEAM_CONFIG:-}"
RESULT_ROOT="${RESULT_ROOT:-${REPO_ROOT}/results/hilevad}"
RUN_NAME="${RUN_NAME:-$(date +%Y%m%d_%H%M%S)}"
RUN_DIR="${RUN_DIR:-${RESULT_ROOT}/${RUN_NAME}}"
SAVE_PATH="${SAVE_PATH:-${RUN_DIR}/viz}"
CHECKPOINT_ENDPOINT="${CHECKPOINT_ENDPOINT:-${RUN_DIR}/results.json}"
DEBUG_CHECKPOINT="${DEBUG_CHECKPOINT:-${RUN_DIR}/live_results.txt}"
PORT="${PORT:-2000}"
TM_PORT="${TM_PORT:-8000}"
GPU_RANK="${GPU_RANK:-0}"
REPETITIONS="${REPETITIONS:-1}"
DEBUG_CHALLENGE="${DEBUG_CHALLENGE:-0}"
TIMEOUT="${TIMEOUT:-600}"
RECORD_PATH="${RECORD_PATH:-}"
PYTHON_BIN="${PYTHON_BIN:-python}"

[ -f "$ROUTES" ] || die "routes file not found: ${ROUTES}"
[ -n "$TEAM_AGENT" ] || die "set TEAM_AGENT=/path/to/your_agent.py"
[ -f "$TEAM_AGENT" ] || die "TEAM_AGENT file not found: ${TEAM_AGENT}"

mkdir -p "$RUN_DIR" "$SAVE_PATH"
export SAVE_PATH

args=(
  "${REPO_ROOT}/leaderboard/leaderboard/leaderboard_evaluator.py"
  --routes="${ROUTES}"
  --repetitions="${REPETITIONS}"
  --track="${CHALLENGE_TRACK_CODENAME}"
  --checkpoint="${CHECKPOINT_ENDPOINT}"
  --debug-checkpoint="${DEBUG_CHECKPOINT}"
  --agent="${TEAM_AGENT}"
  --agent-config="${TEAM_CONFIG}"
  --debug="${DEBUG_CHALLENGE}"
  --record="${RECORD_PATH}"
  --port="${PORT}"
  --traffic-manager-port="${TM_PORT}"
  --timeout="${TIMEOUT}"
  --gpu-rank="${GPU_RANK}"
)

if [ -n "$ROUTES_SUBSET" ]; then
  args+=(--routes-subset="${ROUTES_SUBSET}")
fi

case "${RESUME:-1}" in
  1|true|True|TRUE|yes|YES)
    args+=(--resume=True)
    ;;
esac

case "${USE_EXISTING_SERVER:-0}" in
  1|true|True|TRUE|yes|YES)
    args+=(--use-existing-server)
    ;;
esac

echo "HiLevAD evaluation"
echo "  repo: ${REPO_ROOT}"
echo "  carla launcher root: ${CARLA_ROOT}"
echo "  routes: ${ROUTES}"
echo "  routes subset: ${ROUTES_SUBSET:-<all>}"
echo "  agent: ${TEAM_AGENT}"
echo "  run dir: ${RUN_DIR}"
echo "  checkpoint: ${CHECKPOINT_ENDPOINT}"
echo "  live checkpoint: ${DEBUG_CHECKPOINT}"
echo "  save path: ${SAVE_PATH}"
echo "  port: ${PORT}, tm port: ${TM_PORT}, gpu rank: ${GPU_RANK}"

CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-${GPU_RANK}}" "${PYTHON_BIN}" "${args[@]}"
