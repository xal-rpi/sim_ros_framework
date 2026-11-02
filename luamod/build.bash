#!/usr/bin/env bash
# Package Lua/JSON, torque maps and all staged native vehicle exports by default.
set -euo pipefail
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
PLATFORM=linux
OUTPUT="${BEAMNG_MOD_DIR:-${SCRIPT_DIR}/dist}/xlab.zip"
EXPLICIT_OUTPUT=false
EXPORTS=()
for arg in "$@"; do
  case "$arg" in
    --platform=windows|--win) PLATFORM=windows ;;
    --platform=linux|--linux) PLATFORM=linux ;;
    --output=*) OUTPUT="${arg#*=}"; EXPLICIT_OUTPUT=true ;;
    --vehicle-export=*) EXPORTS+=("${arg#*=}") ;;
    --help)
      echo 'build.bash [--win|--linux] [--output=ZIP] [--vehicle-export=NAME ...]'
      echo 'Packages all runtime Lua/JSON sources, independent of Git staging.'
      echo 'Destination: --output, then BEAMNG_MOD_DIR/xlab.zip, then local dist/xlab.zip.'
      echo 'Default: compile all staged exports under lib/vehicles/<vehicle>.'
      echo '--vehicle-export=NAME (repeatable) restricts the build to an explicit subset.'
      exit 0 ;;
    *) echo "Unknown option: $arg" >&2; exit 1 ;;
  esac
done
OUTPUT="$(realpath -m "$OUTPUT")"
if [[ -z "${BEAMNG_MOD_DIR:-}" && "$EXPLICIT_OUTPUT" == false ]]; then
  echo 'Warning: BEAMNG_MOD_DIR is unset; building into local luamod/dist/. The mod is not installed automatically.' >&2
fi
mkdir -p "$(dirname "$OUTPUT")"
cd "$SCRIPT_DIR"
LIB_DIR=lua/vehicle/controller/xlab/lib
# All staged vehicle libraries coexist in the mod. Runtime vehicle configuration
# chooses which one to FFI-load; changing vehicle needs no rebuild if it is included.
# Explicit selection is only an optional build subset, never required for normal use.
if [[ ${#EXPORTS[@]} -eq 0 ]]; then
  for directory in "$LIB_DIR/vehicles"/*; do
    [[ -d "$directory" ]] || continue
    EXPORTS+=("$(basename "$directory")")
  done
fi
printf 'Building xlab: target=%s\n' "$PLATFORM"
if [[ ${#EXPORTS[@]} -gt 0 ]]; then
  printf 'Native vehicle exports (%s): %s\n' "${#EXPORTS[@]}" "${EXPORTS[*]}"
else
  printf 'Native vehicle exports: none staged under %s/vehicles\n' "$LIB_DIR"
fi
STAGE="$(mktemp -d)"
TEMP_ZIP=''
trap 'rm -rf "$STAGE"; [[ -z "$TEMP_ZIP" ]] || rm -f "$TEMP_ZIP"' EXIT

# Git staging must not change the runtime dependency set in the package.
mapfile -t FILES < <(find lua -type f \( -name '*.lua' -o -name '*.json' \) | sort)
[[ ${#FILES[@]} -gt 0 ]] || { echo 'No Lua/JSON sources found' >&2; exit 1; }
for file in "${FILES[@]}"; do
  # Native export JSON is an input to compilation, not a Lua runtime asset.
  [[ "$file" == *"/lib/vehicles/"* || "$file" == *"/lib/vehicle_runtime/"* ]] && continue
  mkdir -p "$STAGE/$(dirname "$file")"
  cp "$file" "$STAGE/$file"
done

POLICY_DIR="$LIB_DIR/policies"
mkdir -p "$STAGE/$LIB_DIR"
COMPILER=cc
EXT=so
COMPILE_FLAGS=(-O3 -shared -fPIC)
if [[ "$PLATFORM" == windows ]]; then
  COMPILER=x86_64-w64-mingw32-gcc
  EXT=dll
  COMPILE_FLAGS=(-D_WIN32 -O3 -shared)
fi
for src in "$POLICY_DIR"/*.c; do
  [[ -f "$src" ]] || continue
  printf 'Compiling torque/policy library: %s.%s\n' "$(basename "$src" .c)" "$EXT"
  "$COMPILER" "${COMPILE_FLAGS[@]}" \
    -I"$POLICY_DIR" -o "$STAGE/$LIB_DIR/$(basename "$src" .c).$EXT" "$src" -lm
done

# The compiled bytes, not the model source name, identify the runtime cache.
python3 - "$STAGE/$LIB_DIR" "$PLATFORM" "$EXT" <<'PYTORQUE'
import hashlib
import json
from pathlib import Path
import sys

folder, target, extension = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
for library in sorted(folder.glob("*." + extension)):
    record = {
        "target": target,
        "library": library.name,
        "library_sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
    }
    library.with_suffix(".manifest.json").write_text(json.dumps(record, indent=2) + "\n")
PYTORQUE

# Compile staged sources here; model generation belongs to drive_trajopt.
for vehicle in "${EXPORTS[@]}"; do
  printf 'Compiling native vehicle export: %s\n' "$vehicle"
  python3 "$SCRIPT_DIR/build_vehicle.py" --vehicle "$vehicle" --target "$PLATFORM" \
    --output "$STAGE/$LIB_DIR/vehicles"
done

(
  cd "$STAGE"
  zip -qr package.zip lua
)
# Copy to a sibling temporary file; replace the destination only after success.
TEMP_ZIP="$(mktemp "${OUTPUT}.XXXXXX")"
cp "$STAGE/package.zip" "$TEMP_ZIP"
mv -f "$TEMP_ZIP" "$OUTPUT"
printf 'Built %s (%s)\n' "$OUTPUT" "$PLATFORM"
