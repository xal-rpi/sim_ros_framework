#!/usr/bin/env bash
set -euo pipefail

GIT_ROOT=$(git rev-parse --show-toplevel)
cd "${GIT_ROOT}/luamod"

# target zip
MOD_DIR="${HOME}/.local/share/BeamNG.drive/0.35/mods"
MOD_ZIP="${MOD_DIR}/xlab.zip"

# ensure target dirs exist
mkdir -p "$(dirname "$MOD_DIR")"

# remove any existing zip
rm -f "$MOD_ZIP"

# list of globs or paths you want in the mod
declare -a WANT="*.lua"

# collect only tracked files matching WANT
mapfile -t FILES < <(git ls-files "${WANT[@]}")

# if nothing to do, exit
if [ ${#FILES[@]} -eq 0 ]; then
  echo "No matching tracked files found." >&2
  exit 1
fi

# zip them preserving paths
printf '%s\n' "${FILES[@]}" | zip -q "$MOD_ZIP" -@

echo "Wrote ${#FILES[@]} files to $MOD_ZIP"
