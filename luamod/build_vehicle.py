"""Compile an explicitly staged float32 vehicle export for the Lua mod.

Build host: Ubuntu/WSL, Python 3.10. Linux uses cc; Windows x86_64 uses
x86_64-w64-mingw32-gcc. Native Windows/MSVC hosting is not implemented.
No drive_trajopt, CasADi or checkpoint dependency. Source staging is owned by
drive_trajopt.embedded.build --stage-luamod. This script owns target compilation.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent
LIB = ROOT / "lua/vehicle/controller/xlab/lib"
COMMON_SOURCES = (
    "projection.c",
    "vehicle_surface.c",
    "vehicle_environment.c",
    "roll_model.c",
    "roll_filter.c",
)


def build(vehicle: str, target: str, output: Path) -> Path:
    """Verify source identities and build one library + matching FFI header.

    output is the parent vehicles directory in the temporary package tree.
    Existing vehicle outputs are rejected. Compilation failures abort packaging.
    """
    if target not in ("linux", "windows"):
        raise ValueError("Target must be linux or windows")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", vehicle):
        raise ValueError("Vehicle must be a staged export identifier")
    source = LIB / "vehicles" / vehicle
    record = json.loads((source / "export.json").read_text())
    if (
        record["vehicle"],
        record["precision"],
        record["abi_version"],
        record["environment_abi_version"],
    ) != (vehicle, "float32", 4, 1):
        raise ValueError("Incompatible vehicle source export")
    for relative, expected in record["source_sha256"].items():
        path = (LIB / relative).resolve()
        if not path.is_relative_to(LIB.resolve()):
            raise ValueError("Source manifest path outside native library tree")
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(
                f"Source changed since staging: {relative}; restage the export"
            )
    common = LIB / "vehicle_runtime"
    generated = source / "generated"
    sources = [
        generated / name for name in ("geometry_generated.c", "barrier_generated.c")
    ]
    sources += [common / name for name in COMMON_SOURCES]
    required = [
        *sources,
        *common.glob("*.h"),
        *generated.glob("*.h"),
        source / "roll_filter_ffi.h",
    ]
    for path in required:
        if str(path.relative_to(LIB)) not in record["source_sha256"]:
            raise ValueError(f"Unrecorded build input: {path}")
    destination = output / vehicle
    destination.mkdir(parents=True, exist_ok=False)
    windows = target == "windows"
    compiler = "x86_64-w64-mingw32-gcc" if windows else "cc"
    library = destination / ("roll_filter.dll" if windows else "libroll_filter.so")
    command = [
        compiler,
        "-std=c99",
        "-O2",
        "-shared",
        "-DCR_FLOAT32",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-Wdouble-promotion",
        "-Wno-unused-parameter",
        "-Wl,--no-undefined",
        *(["-Wl,--export-all-symbols"] if windows else ["-fPIC"]),
        "-I",
        str(generated),
        "-I",
        str(common),
        *(str(path) for path in sources),
        "-lm",
        "-o",
        str(library),
    ]
    subprocess.run(command, check=True, timeout=90)
    header = destination / "roll_filter_ffi.h"
    shutil.copyfile(source / header.name, header)
    record.update(
        target=target,
        library=library.name,
        compiler=subprocess.check_output(
            [compiler, "--version"], text=True
        ).splitlines()[0],
        library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
        ffi_sha256=hashlib.sha256(header.read_bytes()).hexdigest(),
    )
    (destination / "manifest.json").write_text(json.dumps(record, indent=2) + "\n")
    print(f"Compiled float32 vehicle export: {vehicle} ({target})")
    return library


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", required=True)
    parser.add_argument("--target", choices=("linux", "windows"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    build(args.vehicle, args.target, args.output)
