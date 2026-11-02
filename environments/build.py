#!/usr/bin/env python3
"""Package assets and a checksum manifest into BEAMNG_MOD_DIR or local dist/."""

import argparse
import hashlib
import json
import os
import sys
import tempfile
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo

ROOT = Path(__file__).resolve().parent


def build(output: Path) -> None:
    manifest = json.loads((ROOT / "package.json").read_text())
    files = {
        path.relative_to(ROOT / "mod").as_posix(): path.read_bytes()
        for path in sorted((ROOT / "mod").rglob("*")) if path.is_file()
    }
    for asset in manifest["assets"].values():
        asset["sha256"] = hashlib.sha256(files[asset["path"].lstrip("/")]).hexdigest()
    files["art/xlab_environments/manifest.json"] = (
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    ).encode()
    output.parent.mkdir(parents=True, exist_ok=True)
    # A failed build must not truncate a previously installed, working mod.
    with tempfile.NamedTemporaryFile(dir=output.parent, prefix=output.name + ".", delete=False) as temp:
        temporary = Path(temp.name)
    try:
        # Fixed ZIP timestamps make identical sources produce identical artifacts.
        with ZipFile(temporary, "w") as archive:
            for name, contents in sorted(files.items()):
                info = ZipInfo(name, date_time=(2020, 1, 1, 0, 0, 0))
                info.compress_type = ZIP_DEFLATED
                info.external_attr = 0o644 << 16
                archive.writestr(info, contents)
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"Built {output} ({len(manifest['assets'])} assets)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mod_dir = os.environ.get("BEAMNG_MOD_DIR")
    default_output = (
        Path(mod_dir).expanduser() / "xlab_environments.zip"
        if mod_dir else ROOT / "dist/xlab_environments.zip"
    )
    parser.add_argument(
        "--output", type=Path,
        help="ZIP destination; overrides BEAMNG_MOD_DIR (otherwise uses local dist/)",
    )
    args = parser.parse_args()
    if not mod_dir and args.output is None:
        print(
            "Warning: BEAMNG_MOD_DIR is unset; building into local environments/dist/. "
            "The mod is not installed automatically. Set BEAMNG_MOD_DIR to your "
            "active mods folder or copy the ZIP there.",
            file=sys.stderr,
        )
    build(args.output if args.output is not None else default_output)
