#!/usr/bin/env bash
#
# .git/hooks/pre-commit
# Auto-bump version of your bng_xal packages based on what's staged.
#

set -e

# Find which bng_xal packages have staged changes
pkgs=$(git diff --cached --name-only \
       | grep -E '^src/bng_xal/([^/]+)/' \
       | sed -E 's|src/bng_xal/([^/]+)/.*|\1|' \
       | sort -u)

# If none of our packages changed, skip bump logic
if [[ -z "$pkgs" ]]; then
  exit 0
fi

echo "Detected changes in: $pkgs"

# Prompt user to choose one pkg (or “all”)
echo
PS3="Select package to bump: "
options=($pkgs all)
select sel_pkg in "${options[@]}"; do
  if [[ -n "$sel_pkg" ]]; then
    break
  fi
done

# Prompt for bump level
echo
PS3="Select bump level: "
bump_opts=(patch minor major)
select sel_level in "${bump_opts[@]}"; do
  if [[ -n "$sel_level" ]]; then
    break
  fi
done

echo
echo "-> Will bump '$sel_pkg' with level '$sel_level'"

# Determine the config files
ROOT="$(git rev-parse --show-toplevel)"
declare -A CFG
CFG[bng_controller]="$ROOT/src/bng_xal/bng_controller/.bumpversion.cfg"
CFG[bng_simulator]="$ROOT/src/bng_xal/bng_simulator/.bumpversion.cfg"
CFG[bng_msgs]="$ROOT/src/bng_xal/bng_msgs/.bumpversion.cfg"

# Helper to run bump2version --no-commit/--no-tag
run_bump(){
  local pkg=$1 part=$2
  local cfg="${CFG[$pkg]}"
  if [[ ! -f $cfg ]]; then
    echo "ERROR: no bump config for package '$pkg'" >&2
    exit 1
  fi
  (
  cd "$ROOT/src/bng_xal/$p/"
  bump2version "$part" \
    --config-file "$cfg" \
    --no-commit --no-tag --allow-dirty
  )
}

if [[ "$sel_pkg" == "all" ]]; then
  # bump each package but do not commit or tag yet
  for p in bng_controller bng_simulator bng_msgs; do
    echo "Bumping $p..."
    run_bump "$p" "$sel_level"
  done
  # now stage everything
  git add "$ROOT/src/bng_xal/bng_controller/{setup.py,package.xml}" \
          "$ROOT/src/bng_xal/bng_simulator/{setup.py,package.xml}" \
          "$ROOT/src/bng_xal/bng_msgs/package.xml"
else
  # bump only the selected package
  run_bump "$sel_pkg" "$sel_level"
  # stage its files
  case "$sel_pkg" in
    bng_controller|bng_simulator)
      git add "$ROOT/src/bng_xal/$sel_pkg/setup.py" \
              "$ROOT/src/bng_xal/$sel_pkg/package.xml"
      ;;
    bng_msgs)
      git add "$ROOT/src/bng_xal/$sel_pkg/package.xml"
      ;;
  esac
fi

echo "Version bump staged. Completing your commit…"
exit 0
