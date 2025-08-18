#!/usr/bin/env bash
set -euo pipefail

# # 📦 XLab Mod Packaging Script

# This Bash script automates packaging your BeamNG.drive mod (`xlab.zip`) from tracked or all source files in your Git repository. It ensures platform-specific compilation, includes relevant Lua and JSON files, and outputs a ready-to-use zip file.

# ---

# ## Features

# - Automatically detects Git root and validates mod directory
# - Selects tracked or all model files based on flags
# - Compiles native or Windows DLL for neural net utilities
# - Preserves file paths inside the zip
# - Outputs to BeamNG mod directory


# Resolve absolute path to the script
SCRIPT_PATH="$(readlink -f "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"

# Find the Git root from the script's location
GIT_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"
if [[ -z "$GIT_ROOT" ]]; then
  echo "❌ Error: Script is not inside a Git repository. Please place it within your repo." >&2
  exit 1
fi
cd "${GIT_ROOT}/luamod"

# Default build target
PLATFORM="linux"
MODEL_MODE="tracked"

# Optional platform override
for arg in "$@"; do
  case "$arg" in
    --platform=windows|--win)
      PLATFORM="windows"
      ;;
    --platform=linux|--linux)
      PLATFORM="linux"
      ;;
    --models=all|--all)
      MODEL_MODE="all"
      ;;
    --models=tracked|--tracked)
      MODEL_MODE="tracked"
      ;;
    *)
      echo "Unknown option: $arg"
      exit 1
      ;;
  esac
done

# Print platform
echo "Building for platform: $PLATFORM"
echo "Including models mode: $MODEL_MODE"

# Set default mod directory (Linux)
MOD_DIR="${HOME}/.local/share/BeamNG.drive/0.35/mods"
# Override if BEAMNG_MOD_DIR is defined
if [[ -n "${BEAMNG_MOD_DIR:-}" ]]; then
  MOD_DIR="${BEAMNG_MOD_DIR}"
fi

# Validate that MOD_DIR exists
if [[ ! -d "$MOD_DIR" ]]; then
  echo "❌ Error: Mod directory '$MOD_DIR' does not exist." >&2
  echo "Please create it manually or set BEAMNG_MOD_DIR to a valid path." >&2
  echo ""
  echo "📦 Example for WSL users:"
  echo '  mkdir -p /mnt/c/Users/<your-username>/AppData/Local/BeamNG.drive/0.35/mods'
  echo '  export BEAMNG_MOD_DIR="/mnt/c/Users/<your-username>/AppData/Local/BeamNG.drive/0.35/mods"'
  echo ""
  echo "🔁 To make this permanent, run:"
  echo "  echo 'export BEAMNG_MOD_DIR=\"/mnt/c/Users/<your-username>/AppData/Local/BeamNG.drive/0.35/mods\"' >> ~/.bashrc"
  echo "  source ~/.bashrc"
  exit 1
fi

MOD_ZIP="${MOD_DIR}/xlab.zip"

# ensure target dirs exist
mkdir -p "$(dirname "$MOD_DIR")"

# remove any existing zip
rm -f "$MOD_ZIP"

# list of globs or paths you want in the mod
declare -a WANT="*.lua"
if [[ "$MODEL_MODE" == "tracked" ]]; then
  mapfile -t FILES < <(git ls-files "${WANT[@]}")
else
  mapfile -t FILES < <(find . -type f \( -name "${WANT[0]}" \))
fi

# if nothing to do, exit
if [ ${#FILES[@]} -eq 0 ]; then
  echo "No matching tracked files found." >&2
  exit 1
fi

# Compile nn util
NN_DIR="./lua/vehicle/controller/xlab/lib"
cc -O3 -fPIC -shared -o ${NN_DIR}/libnn.so ${NN_DIR}/nn.c -lm
if [[ "$PLATFORM" == "windows" ]]; then
  if ! command -v x86_64-w64-mingw32-gcc &> /dev/null; then
    echo "❌ Error: MinGW toolchain not found. Please install x86_64-w64-mingw32-gcc." >&2
    exit 1
  fi
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
if [[ "$MODEL_MODE" == "tracked" ]]; then
  mapfile -t TRACKED_MODELS < <(git ls-files "$MODELS_DIR" | grep '\.json$')
  FILES+=( "${TRACKED_MODELS[@]}" )
else
  mapfile -t ALL_MODELS < <(find "$MODELS_DIR" -type f -name "*.json")
  FILES+=( "${ALL_MODELS[@]}" )
fi

# zip them preserving paths
printf '%s\n' "${FILES[@]}" | zip -q "$MOD_ZIP" -@

# Print summary
echo -e "\n📦 Exported ${#FILES[@]} files:"
for f in "${FILES[@]}"; do
  echo "  └── $f"
done

# Show zip contents (outline)
echo -e "\n🗂️ Zip structure:"
unzip -l "$MOD_ZIP" | awk 'NR>3 { print "  └──", $4 }' | sed '$d'

# Highlight destination
echo -e "\n✅ Zip written to: \033[1;31m$MOD_ZIP\033[0m"

