"""Checks for deployed asset integrity and config-selected scene additions."""

import hashlib
import json
import math
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from bng_simulator.core.environment_assets import load_environment_assets

TRACK = "/trackEditor/xlab_environments/donut_r15_b15.json"


def backend(content="{}\n"):
    digest = hashlib.sha256(content.encode()).hexdigest()
    response = {"manifest": {"name": "xlab_environments", "version": "0.1.0",
                "assets": {"donut": {"path": TRACK, "type": "trackbuilder",
                                      "base_level": "tech_ground", "sha256": digest}}},
                "contents": {TRACK: content}}
    beamng = SimpleNamespace(_send=Mock(), scenario=SimpleNamespace(
        load_trackbuilder_track=Mock()))
    beamng._send.return_value.recv.return_value = response
    config = {"scenario": {"level": "tech_ground"},
              "environment": {"trackbuilder_tracks": [TRACK]}}
    return beamng, config, response


def test_load_records_deployed_asset_identity():
    beamng, config, response = backend()
    load_environment_assets(beamng, config, Mock())
    beamng.scenario.load_trackbuilder_track.assert_called_once_with(TRACK)
    recorded = config["environment"]["loaded_assets"][0]
    assert recorded["sha256"] == response["manifest"]["assets"]["donut"]["sha256"]
    assert recorded["package"] == "xlab_environments"
    assert recorded["version"] == "0.1.0"


def test_plain_level_needs_no_environment_mod():
    beamng, _, _ = backend()
    load_environment_assets(beamng, {"scenario": {"level": "tech_ground"}}, Mock())
    beamng._send.assert_not_called()
    beamng.scenario.load_trackbuilder_track.assert_not_called()


@pytest.mark.parametrize("paths", [
    [TRACK, TRACK], ["/trackEditor/xlab_environments/../other.json"],
    ["/levels/tech_ground/mainLevel.lua"], "not-a-list",
])
def test_bad_paths_fail_before_server_queries(paths):
    beamng, config, _ = backend()
    config["environment"]["trackbuilder_tracks"] = paths
    with pytest.raises(ValueError):
        load_environment_assets(beamng, config, Mock())
    beamng._send.assert_not_called()


@pytest.mark.parametrize("failure", ["checksum", "level", "manifest"])
def test_bad_deployment_never_spawns_tracks(failure):
    beamng, config, response = backend()
    if failure == "checksum":
        response["contents"][TRACK] = '{"modified":true}'
    elif failure == "level":
        config["scenario"]["level"] = "derby"
    else:
        response["manifest"]["assets"] = {}
    with pytest.raises(ValueError):
        load_environment_assets(beamng, config, Mock())
    beamng.scenario.load_trackbuilder_track.assert_not_called()


def test_oval_preset_preserves_base_level(monkeypatch):
    import bng_simulator.utils.scenario_compose as compose

    config_dir = Path(__file__).resolve().parents[2] / "bng_bringup/config"
    monkeypatch.setattr(compose, "bringup_config_dir", lambda: str(config_dir))
    result = compose.compose_scenario("tech_ground_oval.yaml")
    assert result["scenario"]["level"] == "tech_ground"
    assert result["environment"]["trackbuilder_tracks"] == [TRACK]
    assert result["vehicles"]["EGO"]["spawn"]["pos"] == [34, -30, 5.5]
    assert result["vehicles"]["EGO"]["spawn"]["cling"] is False
    plain = compose.compose_scenario("single_vehicle.yaml")
    assert "environment" not in plain


@pytest.mark.parametrize("asset,preset,bank", [
    ("flat_circuit", "flat_circuit", 0),
    ("rolling_hills_circuit", "rolling_hills_circuit", 0),
    ("banked_oval_b15", "banked_oval", -15),
])
def test_imported_track_spawn_matches_first_straight_and_manifest(monkeypatch, asset, preset, bank):
    """Catch wrong asset wiring, grid scaling, heading, or ground clinging."""
    import bng_simulator.utils.scenario_compose as compose

    config_dir = Path(__file__).resolve().parents[2] / "bng_bringup/config"
    root = next(parent for parent in config_dir.parents if (parent / "environments/package.json").is_file())
    monkeypatch.setattr(compose, "bringup_config_dir", lambda: str(config_dir))
    overrides = {
        "vehicle": "utv_canam_x3_r322_easylift",
        "level": f"tech_ground_with_{preset}",
        "spawn": "on_track",
    }
    result = compose.compose_scenario("tech_ground_oval.yaml", overrides)
    manifest = json.loads((root / "environments/package.json").read_text())
    entry = manifest["assets"][asset]
    track = json.loads((root / "environments/mod" / entry["path"].lstrip("/")).read_text())
    assert result["scenario"]["level"] == entry["base_level"] == track["level"]
    assert result["environment"]["trackbuilder_tracks"] == [entry["path"]]
    vehicle = result["vehicles"]["EGO"]
    assert vehicle["model_args"]["part_config"] == "/vehicles/utv/canam_x3_r322_easylift.pc"
    spawn = vehicle["spawn"]
    start = track["subTrackPositions"][0]
    initial, straight = track["subPieces"][0][:2]
    dx, dy = spawn["pos"][0] - 4*start["x"], spawn["pos"][1] - 4*start["y"]
    along = dx*math.sin(start["hdg"]) + dy*math.cos(start["hdg"])
    across = dx*math.cos(start["hdg"]) - dy*math.sin(start["hdg"])
    assert straight["piece"] == "freeForward"
    assert along == pytest.approx(5, abs=0.001)
    assert abs(across) < 0.001
    assert 3 < along < 4*straight["length"] - 3
    assert initial["height"]["value"] == straight.get("height", initial["height"])["value"] == 0
    assert initial["bank"]["value"] == straight.get("bank", initial["bank"])["value"] == bank
    assert spawn["pos"][2] == pytest.approx(start["z"] + 0.5)
    assert spawn["xlab_yaw_deg"] == pytest.approx(90 - math.degrees(start["hdg"]), abs=0.006)
    assert spawn["yaw_angle"] == pytest.approx(-spawn["xlab_yaw_deg"] - 90)
    assert spawn["cling"] is False
    # CLI position override must reach the initial configuration, not only teleport.
    overridden = compose.compose_scenario("tech_ground_oval.yaml", {**overrides, "pos": [12, 13, 14], "yaw": 25})
    assert overridden["vehicles"]["EGO"]["spawn"]["pos"] == [12, 13, 14]
    assert overridden["vehicles"]["EGO"]["spawn"]["xlab_yaw_deg"] == 25


def test_elevated_spawn_is_restored_with_configured_heading():
    from bng_simulator.core.scenario_builder import ScenarioBuilder

    spawn = {"pos": [32, -30, 5.5], "yaw_angle": -90, "cling": False}
    vehicle = SimpleNamespace(vid="EGO", teleport=Mock(return_value=True))
    manager = SimpleNamespace(vehicle=vehicle, get_spawn_args=lambda: deepcopy(spawn))
    builder = ScenarioBuilder(SimpleNamespace(logger=Mock()))
    builder.restore_explicit_spawn(manager)
    call = vehicle.teleport.call_args
    assert call.args == ((32, -30, 5.5),)
    assert call.kwargs["reset"] is True
    assert call.kwargs["rot_quat"] == pytest.approx((0, 0, -2**-0.5, 2**-0.5))
    assert spawn["yaw_angle"] == -90
    vehicle.teleport.return_value = False
    with pytest.raises(RuntimeError, match="Could not restore explicit spawn"):
        builder.restore_explicit_spawn(manager)


def test_ground_clinging_spawn_is_not_teleported():
    from bng_simulator.core.scenario_builder import ScenarioBuilder

    vehicle = SimpleNamespace(teleport=Mock())
    manager = SimpleNamespace(vehicle=vehicle, get_spawn_args=lambda: {"cling": True})
    ScenarioBuilder(SimpleNamespace(logger=Mock())).restore_explicit_spawn(manager)
    vehicle.teleport.assert_not_called()


@pytest.fixture
def environment_builder():
    import importlib.util

    root = Path(__file__).resolve().parents[4]
    spec = importlib.util.spec_from_file_location("environment_build", root / "environments/build.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_environment_build_is_deterministic_and_records_actual_bytes(environment_builder, tmp_path):
    from zipfile import ZipFile

    first, second = tmp_path / "a.zip", tmp_path / "b.zip"
    environment_builder.build(first)
    environment_builder.build(second)
    assert first.read_bytes() == second.read_bytes()
    with ZipFile(first) as archive:
        manifest = json.loads(archive.read("art/xlab_environments/manifest.json"))
        for asset in manifest["assets"].values():
            assert hashlib.sha256(archive.read(asset["path"].lstrip("/"))).hexdigest() == asset["sha256"]


@pytest.mark.parametrize("failure", ["write", "replace"])
@pytest.mark.parametrize("installed", [True, False])
def test_failed_environment_rebuild_preserves_installed_archive(
    environment_builder, tmp_path, monkeypatch, failure, installed
):
    output = tmp_path / "installed.zip"
    if installed:
        environment_builder.build(output)
    before = output.read_bytes() if installed else None

    def fail(*args, **kwargs):
        raise OSError("injected " + failure + " failure")

    if failure == "write":
        monkeypatch.setattr(environment_builder.ZipFile, "writestr", fail)
    else:
        monkeypatch.setattr(Path, "replace", fail)
    with pytest.raises(OSError, match="injected"):
        environment_builder.build(output)
    assert (output.read_bytes() if output.exists() else None) == before
    assert sorted(p.name for p in tmp_path.iterdir()) == (["installed.zip"] if installed else [])


@pytest.fixture
def lua_package_source(tmp_path):
    """Exercise the real packager outside Git, with one small compiled library."""
    import shutil

    root = Path(__file__).resolve().parents[4]
    source = tmp_path / "luamod"
    source.mkdir()
    shutil.copyfile(root / "luamod/build.bash", source / "build.bash")
    shutil.copytree(root / "luamod/lua", source / "lua", ignore=shutil.ignore_patterns("lib"))
    policies = source / "lua/vehicle/controller/xlab/lib/policies"
    policies.mkdir(parents=True)
    (policies / "test_map.c").write_text("float test_torque(float u) { return 100 * u; }\n")
    return source


@pytest.mark.parametrize("destination", ["default", "environment", "explicit"])
def test_lua_package_includes_untracked_runtime_tree_and_library_identity(lua_package_source, tmp_path, destination):
    import os
    import subprocess
    from zipfile import ZipFile

    source = lua_package_source
    env = dict(os.environ)
    env.pop("BEAMNG_MOD_DIR", None)
    command = ["bash", str(source / "build.bash")]
    output = source / "dist/xlab.zip"
    if destination != "default":
        env["BEAMNG_MOD_DIR"] = str(tmp_path / "mods")
        output = tmp_path / "mods/xlab.zip"
    if destination == "explicit":
        output = tmp_path / "explicit/xlab.zip"
        command.append("--output=" + str(output))
    result = subprocess.run(command, env=env, capture_output=True, text=True, check=True, timeout=30)
    assert ("not installed automatically" in result.stderr) == (destination == "default")
    with ZipFile(output) as archive:
        members = set(archive.namelist())
        for path in (source / "lua").rglob("*"):
            if path.is_file() and path.suffix in (".lua", ".json"):
                assert path.relative_to(source).as_posix() in members
        base = "lua/vehicle/controller/xlab/lib/"
        manifest = json.loads(archive.read(base + "test_map.manifest.json"))
        assert manifest["target"] == "linux"
        assert manifest["library"] == "test_map.so"
        assert manifest["library_sha256"] == hashlib.sha256(archive.read(base + manifest["library"])).hexdigest()
    if destination == "explicit":
        assert not (tmp_path / "mods/xlab.zip").exists()


def test_failed_native_compile_preserves_lua_archive(lua_package_source, tmp_path):
    import subprocess

    output = tmp_path / "installed.zip"
    output.write_bytes(b"existing working mod")
    library = lua_package_source / "lua/vehicle/controller/xlab/lib/policies/test_map.c"
    library.write_text("#error injected compile failure\n")
    result = subprocess.run(
        ["bash", str(lua_package_source / "build.bash"), "--output=" + str(output)],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode != 0
    assert "injected compile failure" in result.stderr
    assert output.read_bytes() == b"existing working mod"
    assert not list(tmp_path.glob("installed.zip.*"))


@pytest.mark.parametrize("option", ["--lua=tracked", "--lua=all", "--tracked", "--all"])
def test_removed_lua_packaging_modes_fail_explicitly(option, tmp_path):
    import subprocess

    root = Path(__file__).resolve().parents[4]
    result = subprocess.run(
        ["bash", str(root / "luamod/build.bash"), option],
        cwd=tmp_path, capture_output=True, text=True, timeout=5,
    )
    assert result.returncode != 0
    assert "Unknown option: " + option in result.stderr
