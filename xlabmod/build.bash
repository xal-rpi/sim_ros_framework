#!/usr/bin/env bash
set -euo pipefail

# target zip
MOD_ZIP="${HOME}/.local/share/BeamNG.drive/0.35/mods/xlab.zip"

# ensure target dir exists
mkdir -p "$(dirname "$MOD_ZIP")"

# remove any existing zip
rm -f "$MOD_ZIP"

# list of globs or paths you want in the mod
# (git ls-files will only output those that are tracked)
declare -a WANT="*.lua"

# collect only tracked files matching WANT
mapfile -t FILES < <(git ls-files "${WANT[@]}")

# if nothing to do, exit
if [ ${#FILES[@]} -eq 0 ]; then
  echo "No matching tracked files found." >&2
  exit 1
fi

# zip them with STORE (no compression), preserving paths
# -0    : store only
# -q    : quiet
# -@    : read file-list from stdin
printf '%s\n' "${FILES[@]}" \
  | zip -q "$MOD_ZIP" -@

echo "Wrote ${#FILES[@]} files to $MOD_ZIP"
