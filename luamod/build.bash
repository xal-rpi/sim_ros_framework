#!/usr/bin/env bash
set -euo pipefail

# Default platform
PLATFORM="linux"

# Check for optional argument
for arg in "$@"; do
  case "$arg" in
    --windows|--platform=windows)
      PLATFORM="windows"
      ;;
    --linux|--platform=linux)
      PLATFORM="linux"
      ;;
    *)
      echo "Unknown option: $arg"
      exit 1
      ;;
  esac
done

GIT_ROOT=$(git rev-parse --show-toplevel)
cd "${GIT_ROOT}/luamod"

# target zip
# Choose mod directory based on platform
if [[ "$PLATFORM" == "windows" ]]; then
  # TODO: Adapt username for Windows
  MOD_DIR="/mnt/c/Users/frmbo/AppData/Local/BeamNG.drive/0.35/mods"
else
  MOD_DIR="${HOME}/.local/share/BeamNG.drive/0.35/mods"
fi
MOD_ZIP="${MOD_DIR}/xlab.zip"

# ensure target dirs exist
mkdir -p "$(dirname "$MOD_DIR")"

# remove any existing zip
rm -f "$MOD_ZIP"

# list of globs or paths you want in the mod
declare -a WANT="*.lua"

# TODO: Disabled "collect only tracked files matching WANT"
# mapfile -t FILES < <(git ls-files "${WANT[@]}")

# collect all matching files (tracked or not)
mapfile -t FILES < <(find . -type f \( -name "${WANT[0]}" \))

# if nothing to do, exit
if [ ${#FILES[@]} -eq 0 ]; then
  echo "No matching tracked files found." >&2
  exit 1
fi

# Compile nn util
NN_DIR="./lua/vehicle/controller/xlab/lib"
cc -O3 -fPIC -shared -o ${NN_DIR}/libnn.so ${NN_DIR}/nn.c -lm
if [[ "$PLATFORM" == "windows" ]]; then
  # TODO: This assumes you have the MinGW toolchain installed on WSL. Better Alternative?
  x86_64-w64-mingw32-gcc -D_WIN32 -O3 -shared -o "${NN_DIR}/libnn.dll" "${NN_DIR}/nn.c" -lm
  FILES+=( "${NN_DIR}/libnn.dll" )
else
  # For Linux, use the native compiler
  cc -O3 -fPIC -shared -o "${NN_DIR}/libnn.so" "${NN_DIR}/nn.c" -lm
  FILES+=( "${NN_DIR}/libnn.so" )
fi

# Add actions
MODELS_DIR="./lua/ge/extensions/core/input/actions"
FILES+=( "${MODELS_DIR}/bypass_controller.json" )

# Add models
MODELS_DIR="./lua/vehicle/controller/xlab/models"
FILES+=( "${MODELS_DIR}/test.json" )
FILES+=( "${MODELS_DIR}/wheel_speed.json" )
FILES+=( "${MODELS_DIR}/wheel_speed_v1.json" )
FILES+=( "${MODELS_DIR}/wheel_speed_v2.json" )

# zip them preserving paths
printf '%s\n' "${FILES[@]}" | zip -q "$MOD_ZIP" -@

echo "Wrote ${#FILES[@]} files to $MOD_ZIP"
