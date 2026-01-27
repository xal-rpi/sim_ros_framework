#!/usr/bin/env bash
# Build ROS 2 workspace with C/C++ msgs first, then Python packages via symlink-install.
set -Eeuo pipefail

# -------- config (edit if your package names differ) --------
PKG_CPP="bng_msgs"                 # C/C++ package (messages)
PKG_LAUNCH="bng_bringup"          # Launch package
PKGS_PY=("bng_simulator" "bng_controller")  # Python packages
# ------------------------------------------------------------

# colors
c_reset="\033[0m"
c_green="\033[32m"
c_yellow="\033[33m"
c_red="\033[31m"
c_blue="\033[34m"

log()   { echo -e "${c_blue}[INFO]${c_reset} $*"; }
ok()    { echo -e "${c_green}[OK]${c_reset}   $*"; }
warn()  { echo -e "${c_yellow}[WARN]${c_reset} $*"; }
err()   { echo -e "${c_red}[ERR]${c_reset}  $*" >&2; }

usage() {
  cat <<EOF
Usage: $(basename "$0") [-w WORKSPACE] [-r ROS_DISTRO] [-j JOBS] [--no-rosdep] [--symlink]

Options:
  -w, --workspace   Path to your colcon workspace (default: ~/ros2_ws)
  -r, --ros-distro  ROS 2 distro to source (e.g., humble, jazzy). If omitted,
                    uses \$ROS_DISTRO if already set in the environment.
  -j, --jobs        Parallel jobs passed to colcon (default: auto)
      --no-rosdep   Skip 'rosdep install' step
      --symlink     Use --symlink-install for Python packages (for development)

Examples:
  $(basename "$0")
  $(basename "$0") -w ~/dev/ros2_ws -r humble -j 8 --symlink
EOF
}

# defaults
WORKSPACE="${HOME}/ros2_ws"
ROS_DISTRO="${ROS_DISTRO:-}"
JOBS=""
RUN_ROSDEP=1
CLEAN_ALL=0
CLEAN_PKGS=()
USE_SYMLINK=0

# parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean) CLEAN_ALL=1; shift;;
    --clean-pkg) CLEAN_PKGS+=("$2"); shift 2;;
    -w|--workspace) WORKSPACE="$2"; shift 2;;
    -r|--ros-distro) ROS_DISTRO="$2"; shift 2;;
    -j|--jobs) JOBS="$2"; shift 2;;
    --no-rosdep) RUN_ROSDEP=0; shift;;
    --symlink) USE_SYMLINK=1; shift;;
    -h|--help) usage; exit 0;;
    *) err "Unknown arg: $1"; usage; exit 2;;
  esac
done

# helper to run commands with nice output
run() {
  local desc="$1"; shift
  log "$desc"
  if "$@"; then
    ok "$desc"
  else
    err "Failed: $desc"
    exit 1
  fi
}

# sanity checks
command -v colcon >/dev/null 2>&1 || { err "colcon not found. Install ROS 2 dev tools first."; exit 1; }
command -v python3 >/dev/null 2>&1 || { err "python3 not found."; exit 1; }

# Enforce colcon to use the output of which python3
export COLCON_PYTHON_EXECUTABLE=$(which python3)

if [[ -z "$ROS_DISTRO" ]]; then
  # try to guess from common installs if not in env
  for d in /opt/ros/*; do
    [[ -d "$d" ]] || continue
    cand=$(basename "$d")
    if [[ -f "/opt/ros/${cand}/setup.bash" ]]; then
      ROS_DISTRO="$cand"
      break
    fi
  done
fi

[[ -n "$ROS_DISTRO" ]] || { err "Could not determine ROS_DISTRO. Pass -r <distro> (e.g., humble, jazzy)."; exit 1; }
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || { err "/opt/ros/${ROS_DISTRO}/setup.bash not found."; exit 1; }

# enter workspace
if [[ ! -d "$WORKSPACE" ]]; then
  warn "Workspace ${WORKSPACE} does not exist. Creating it."
  run "Create workspace directories" mkdir -p "${WORKSPACE}/src"
fi
cd "$WORKSPACE"

if [[ "$CLEAN_ALL" -eq 1 ]]; then
  warn "Cleaning build/ install/ log/"
  rm -rf build/ install/ log/
fi

if [[ "${#CLEAN_PKGS[@]}" -gt 0 ]]; then
  for p in "${CLEAN_PKGS[@]}"; do
    warn "Cleaning package artifacts for: $p"
    rm -rf "build/$p" "install/$p"
  done
fi

# source ROS distro
# shellcheck disable=SC1090
run "Source ROS ${ROS_DISTRO}" bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"

# # optional rosdep (recommended for C/C++ deps)
# if [[ $RUN_ROSDEP -eq 1 ]]; then
#   if command -v rosdep >/dev/null 2>&1; then
#     run "Initialize rosdep (may be already initialized)" bash -c "sudo rosdep init >/dev/null 2>&1 || true; rosdep update"
#     run "Install package dependencies with rosdep" rosdep install --from-paths src --ignore-src -r -y
#   else
#     warn "rosdep not found; skipping dependency installation."
#   fi
# fi

# clean up partial/stale builds if mix of packages changed
# (safe: doesn’t delete sources)
if [[ -d build || -d install || -d log ]]; then
  warn "Existing build/install/log detected. Keeping them. If you see odd errors, consider cleaning:"
  echo "      rm -rf build/ install/ log/"
fi

# verify packages exist in src
missing=0
if [[ ! -d "src/${PKG_CPP}" ]]; then
  warn "C/C++ package '${PKG_CPP}' not found at src/${PKG_CPP}"
  missing=1
fi
for p in "${PKGS_PY[@]}"; do
  if [[ ! -d "src/${p}" ]]; then
    warn "Python package '${p}' not found at src/${p}"
    missing=1
  fi
done
if [[ $missing -eq 1 ]]; then
  warn "Some packages are missing. Make sure they are cloned/added under src/."
fi

# build flags
COLCON_COMMON_ARGS=()
[[ -n "$JOBS" ]] && COLCON_COMMON_ARGS+=("--parallel-workers" "$JOBS")
COLCON_HANDLERS="--event-handlers console_direct+"

# 1) build C/C++ msgs first
run "Build ${PKG_CPP}" \
  colcon build ${COLCON_HANDLERS} "${COLCON_COMMON_ARGS[@]}" --packages-select "${PKG_CPP}"

# source overlay after msgs (ensures generated interfaces are found)
# shellcheck disable=SC1091
run "Source workspace overlay" bash -c "source install/setup.bash"

# 2) build Python packages with optional symlink-install
run "Build bng_bringup --symlink-install" \
  colcon build ${COLCON_HANDLERS} "${COLCON_COMMON_ARGS[@]}" --symlink-install --packages-select "${PKG_LAUNCH}"

if [[ "$USE_SYMLINK" -eq 1 ]]; then
  run "Build Python packages (${PKGS_PY[*]}) with --symlink-install" \
    colcon build ${COLCON_HANDLERS} "${COLCON_COMMON_ARGS[@]}" --symlink-install --packages-select "${PKGS_PY[@]}"
else
  run "Build Python packages (${PKGS_PY[*]})" \
    colcon build ${COLCON_HANDLERS} "${COLCON_COMMON_ARGS[@]}" --packages-select "${PKGS_PY[@]}"
fi

# source final overlay
# shellcheck disable=SC1091
run "Source workspace overlay (final)" bash -c "source install/setup.bash"

# quick verification: check package metadata is visible
verify_pkg() {
  local dist="$1"
  python3 - <<PY
import importlib.metadata as m, sys
name="${dist}"
try:
    ver = m.version(name)
    print(f"{name} version: {ver}")
except m.PackageNotFoundError:
    print(f"WARNING: {name} metadata not found via importlib.metadata", file=sys.stderr)
PY
}

ok "Build complete."
log "Verifying Python package metadata visibility (may warn if not using pip-style metadata)..."
for p in "${PKGS_PY[@]}"; do
  # importlib normalizes underscore/dash; try both
  verify_pkg "${p//_/-}" || true
done

echo -e "${c_green}All done.${c_reset} Remember to source this overlay in new shells:"
echo "  source \"${WORKSPACE}/install/setup.bash\""

# REQ="${WORKSPACE}/src/sim_ros_framework/requirements.txt"
# run "Install Python deps" python3 -m pip install -U pip
# run "Install Python deps" python3 -m pip install -r "$REQ"