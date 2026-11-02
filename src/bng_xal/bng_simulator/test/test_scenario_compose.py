"""Scenario composition and simulator startup/shutdown contracts."""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from bng_simulator.utils.scenario_compose import (
    _parse_pos_launch_value,
    beamng_yaw_from_xlab,
    compose_scenario,
    resolve_config_path,
    resolve_preset_path,
    summarize_config,
)


def test_resolve_config_path_single_vehicle() -> None:
    path = resolve_config_path("single_vehicle.yaml")
    assert path.endswith("runs/single_vehicle.yaml")


def test_compose_single_vehicle_defaults(tmp_path) -> None:
    cfg = compose_scenario("single_vehicle.yaml")
    ego = cfg["vehicles"]["EGO"]
    assert cfg["scenario"]["level"] == "tech_ground"
    assert ego["spawn"]["pos"] == [0, 0, 0]
    assert ego["spawn"]["xlab_yaw_deg"] == 0
    assert ego["io"]["port_index"] == 0
    setup = cfg["beamng"]["setup_funcs"]
    assert "beamng.settings.set_nondeterministic" not in setup
    assert setup["beamng.settings.set_deterministic"] == {"speed_factor": -1}

    fixed_run = tmp_path / "fixed.yaml"
    fixed_run.write_text("""compose:
  vehicle: covet_turbo_rally
  level: tech_ground
  spawn: origin
  sim: defaults/sim_fixed_step.yaml
scenario_mode: create
""")
    fixed = compose_scenario(str(fixed_run))["beamng"]["setup_funcs"]
    assert "beamng.settings.set_nondeterministic" not in fixed
    assert fixed["beamng.settings.set_deterministic"] == {
        "steps_per_second": 60, "speed_factor": -1,
    }

    wallclock_run = tmp_path / "wallclock.yaml"
    wallclock_run.write_text("""compose:
  vehicle: covet_turbo_rally
  level: tech_ground
  spawn: origin
scenario_mode: create
overrides:
  beamng:
    setup_funcs:
      beamng.settings.set_deterministic:
        speed_factor: 0
""")
    wallclock = compose_scenario(str(wallclock_run))["beamng"]["setup_funcs"]
    assert "beamng.settings.set_nondeterministic" not in wallclock
    assert wallclock["beamng.settings.set_deterministic"] == {"speed_factor": 0}


def test_spawn_easylift_before_steering_export(monkeypatch) -> None:
    import bng_simulator.utils.scenario_compose as sc

    load_fragment = sc._load_fragment

    def calibration_catalog(fragment):
        data = load_fragment(fragment)
        if fragment == "vehicle_catalog.yaml":
            entry = data["catalog"]["utv_canam_x3_r322_easylift"]
            entry["calibration_only"] = True
            entry.pop("steering_calibration")
        return data

    monkeypatch.setattr(sc, "_load_fragment", calibration_catalog)
    cfg = compose_scenario(
        "single_vehicle.yaml", {"vehicle": "utv_canam_x3_r322_easylift"}
    )
    ego = cfg["vehicles"]["EGO"]
    assert ego["model_args"]["model"] == "utv"
    assert ego["model_args"]["part_config"] == "/vehicles/utv/canam_x3_r322_easylift.pc"
    assert ego["spawn"]["pos"] == [0, 0, 0]
    assert "LowLevelController" not in ego.get("controllers", {})
    assert "gtstate" in ego["sensors"]
    assert "torque_map" not in ego["sensors"]["gtstate"]


def test_normal_control_still_requires_steering_export(monkeypatch) -> None:
    import bng_simulator.utils.scenario_compose as sc

    load_fragment = sc._load_fragment

    def without_calibration_mode(fragment):
        data = load_fragment(fragment)
        if fragment == "vehicle_catalog.yaml":
            data["catalog"]["utv_canam_x3_r322_easylift"].pop("steering_calibration")
        return data

    monkeypatch.setattr(sc, "_load_fragment", without_calibration_mode)
    with pytest.raises(KeyError, match="steering_calibration"):
        compose_scenario("single_vehicle.yaml", {"vehicle": "utv_canam_x3_r322_easylift"})


def test_compose_preset_overrides_level_and_spawn() -> None:
    cfg = compose_scenario("single_vehicle.yaml", {"preset": "derby_grid_lane.yaml"})
    ego = cfg["vehicles"]["EGO"]
    assert cfg["scenario"]["level"] == "derby"
    assert ego["spawn"]["pos"] == pytest.approx([-0.243, 148.0445, 79.3])


def test_compose_preset_then_launch_yaw_wins(sample_preset: Path) -> None:
    cfg = compose_scenario(
        "single_vehicle.yaml",
        {"preset": sample_preset.name, "yaw": 45},
    )
    assert cfg["vehicles"]["EGO"]["spawn"]["xlab_yaw_deg"] == 45.0


def test_compose_launch_pos_override() -> None:
    cfg = compose_scenario(
        "single_vehicle.yaml",
        {
            "level": "derby",
            "spawn": "grid_lane",
            "pos": [1.0, 2.0, 3.0],
        },
    )
    assert cfg["vehicles"]["EGO"]["spawn"]["pos"] == [1.0, 2.0, 3.0]


def test_compose_multi_agent_ports_and_offset() -> None:
    cfg = compose_scenario("multi_agent.yaml")
    ego = cfg["vehicles"]["EGO"]
    agent2 = cfg["vehicles"]["AGENT2"]
    assert ego["io"]["port_index"] == 0
    assert agent2["io"]["port_index"] == 1
    assert agent2["spawn"]["pos"] == [6.0, 0.0, 0.0]
    assert agent2["spawn"]["xlab_yaw_deg"] == -90.0


def test_compose_yaw_offset() -> None:
    cfg = compose_scenario("single_vehicle.yaml")
    spawn = cfg["vehicles"]["EGO"]["spawn"]
    # Zero ENU heading keeps the native -90 degree yaw.
    assert spawn["yaw_angle"] == beamng_yaw_from_xlab(0.0, 90.0)


def test_unknown_spawn_raises() -> None:
    with pytest.raises(KeyError, match="Unknown spawn preset"):
        compose_scenario("single_vehicle.yaml", {"spawn": "does_not_exist"})


def test_launch_override_rejected_for_multi_vehicle() -> None:
    with pytest.raises(ValueError, match="single-vehicle shorthand"):
        compose_scenario("multi_agent.yaml", {"spawn": "origin"})


def test_preset_rejected_for_multi_vehicle() -> None:
    with pytest.raises(ValueError, match="user preset"):
        compose_scenario("multi_agent.yaml", {"preset": "derby_grid_lane.yaml"})


def test_parse_pos_launch_value() -> None:
    assert _parse_pos_launch_value("1.5,2,3.0") == [1.5, 2.0, 3.0]


def test_parse_pos_launch_value_invalid() -> None:
    with pytest.raises(ValueError, match="x,y,z"):
        _parse_pos_launch_value("1,2")


def test_resolve_preset_path_installed_example() -> None:
    path = resolve_preset_path("derby_grid_lane.yaml")
    assert path.endswith("presets/user/derby_grid_lane.yaml")


def test_resolve_preset_path_absolute(tmp_path: Path) -> None:
    preset = tmp_path / "my_spot.yaml"
    preset.write_text("level: derby\nspawn: grid_lane\n", encoding="utf-8")
    assert resolve_preset_path(str(preset)) == str(preset)


def test_compose_custom_preset_absolute_pos(tmp_preset_dir: Path) -> None:
    preset = tmp_preset_dir / "abs_pos.yaml"
    preset.write_text(
        textwrap.dedent("""\
            level: derby
            spawn: grid_lane
            pos: [10.0, 20.0, 30.0]
            """),
        encoding="utf-8",
    )
    cfg = compose_scenario("single_vehicle.yaml", {"preset": "abs_pos.yaml"})
    assert cfg["vehicles"]["EGO"]["spawn"]["pos"] == [10.0, 20.0, 30.0]


def test_summarize_config_single_vehicle() -> None:
    cfg = compose_scenario("single_vehicle.yaml")
    summary = summarize_config(cfg, config_path="single_vehicle.yaml")
    assert "=== xlab scenario manifest ===" in summary
    assert "vehicle EGO" in summary
    assert "tech_ground" in summary
    assert "/EGO/control/cmd" in summary
    assert "control_state@" in summary


def test_summarize_config_launch_overrides() -> None:
    cfg = compose_scenario("single_vehicle.yaml", {"yaw": 45, "level": "derby"})
    summary = summarize_config(
        cfg,
        config_path="single_vehicle.yaml",
        launch_overrides={"yaw": 45, "level": "derby"},
    )
    assert "yaw=45" in summary
    assert "level=derby" in summary


@pytest.mark.parametrize(
    "vehicle", ["covet_dunes", "covet_turbo_rally", "racetruck_tt1_unlimited"]
)
def test_compose_new_vehicle_torque_maps(vehicle):
    cfg = compose_scenario("single_vehicle.yaml", {"vehicle": vehicle})
    ego = cfg["vehicles"]["EGO"]
    controller = ego["controllers"]["LowLevelController"]
    assert controller["calibration"]["torque_map"] == vehicle + "_wheel_torque"
    assert "torque_map_api" not in controller["calibration"]
    assert (
        ego["sensors"]["gtstate"]["torque_map"]["field_name"] == "rear_wheel_torque_est"
    )
    assert controller["drivetrain"]["mode"] == 2
    assert controller["drivetrain"]["startGear"] is None


@pytest.mark.parametrize(
    "vehicle", ["covet_dunes", "covet_turbo_rally", "racetruck_tt1_unlimited"]
)
def test_measured_steering_map_matches_export_and_lua_conversion(vehicle, tmp_path):
    import subprocess
    import yaml
    from bng_simulator.utils.scenario_compose import bringup_config_dir

    cfg = compose_scenario("single_vehicle.yaml", {"vehicle": vehicle})
    calibration = cfg["vehicles"]["EGO"]["controllers"]["LowLevelController"][
        "calibration"
    ]
    samples = calibration["steering_map"]
    catalog = yaml.safe_load(
        (Path(bringup_config_dir()) / "vehicle_catalog.yaml").read_text()
    )
    exported = yaml.safe_load(
        (
            Path(bringup_config_dir())
            / catalog["catalog"][vehicle]["steering_calibration"]
        ).read_text()
    )
    assert samples == [
        {"u": row["u"], "delta": row["delta"]}
        for row in exported["front_steering_samples"]
    ]
    assert "steering_to_input" not in calibration.get("gains", {})
    module = (
        Path(__file__).resolve().parents[4]
        / "luamod/lua/vehicle/controller/xlab/steering_map.lua"
    )
    rows = ",".join("{u=%r,delta=%r}" % (p["u"], p["delta"]) for p in samples)
    script = tmp_path / "steering.lua"
    script.write_text("local samples={" + rows + "}\n" + r"""
local factory=dofile(arg[1])
local map=factory.new(samples)
local function near(a,b) assert(math.abs(a-b)<1e-9,tostring(a)..' ~= '..tostring(b)) end
for i,p in ipairs(samples) do
  near(map.angle(p.u),p.delta);near(map.input(p.delta),p.u)
  if i>1 then
    local previous=samples[i-1]
    near(map.angle((previous.u+p.u)/2),(previous.delta+p.delta)/2)
    near(map.input((previous.delta+p.delta)/2),(previous.u+p.u)/2)
  end
end
near(map.minimum,math.min(samples[1].delta,samples[#samples].delta))
near(map.maximum,math.max(samples[1].delta,samples[#samples].delta))
near(map.angle(-2),samples[1].delta);near(map.angle(2),samples[#samples].delta)
near(map.angle(map.input(map.minimum-1)),map.minimum)
near(map.angle(map.input(map.maximum+1)),map.maximum)
assert(not pcall(factory.new,nil));assert(not pcall(factory.new,{}))
assert(not pcall(factory.new,{{u=-1,delta=0},{u=1,delta=0}}))
assert(not pcall(factory.new,{{u=-1,delta=1},{u=0,delta=-1},{u=1,delta=0}}))
assert(not pcall(factory.new,{{u=-1,delta=1},{u=0,delta=0}}))
assert(not pcall(map.angle,0/0));assert(not pcall(map.input,math.huge))
local positive=factory.new({{u=-1,delta=-.4},{u=0,delta=.01},{u=1,delta=.7}})
near(positive.angle(-1),-.4);near(positive.input(.7),1)
input={steering=0,throttle=0,brake=0,parkingbrake=0}
input.updateGFX=function() input.steering=-1 end
electrics={values={}}
local manual=dofile(arg[2])
manual.init({steering_rate_rad_s=.2},map)
manual.beginManual(map.input(0),0)
input.updateGFX(.01)
near(map.angle(manual.getTargets(.01).steering),.002)
manual.stop()
print('PASS measured steering map: knots, interpolation, inverse, asymmetry, saturation and invalid data')
""")
    subprocess.run(
        [
            "texluajit",
            "--luaonly",
            str(script),
            str(module),
            str(module.with_name("manual_input.lua")),
        ],
        check=True,
    )


def test_unsupported_vehicle_and_invalid_steering_map_fail(tmp_path):
    import yaml
    from bng_simulator.utils.scenario_compose import _apply_catalog_to_vehicle

    with pytest.raises(KeyError, match="Unknown vehicle"):
        compose_scenario("single_vehicle.yaml", {"vehicle": "unknown_vehicle"})
    path = tmp_path / "bad.yaml"
    for samples in [
        [],
        [{"u": -1, "delta": 1}, {"u": 1, "delta": 1}],
        [{"u": -1, "delta": 1}, {"u": 0, "delta": 0}],
        [{"u": -1, "delta": 1}, {"u": 0, "delta": -1}, {"u": 1, "delta": 0}],
    ]:
        path.write_text(yaml.safe_dump({"front_steering_samples": samples}))
        with pytest.raises(ValueError):
            _apply_catalog_to_vehicle({}, {"steering_calibration": str(path)})


def test_easylift_uses_measured_steering_and_shadow_filter():
    cfg = compose_scenario("single_vehicle.yaml", {"vehicle": "utv_canam_x3_r322_easylift"})
    llc = cfg["vehicles"]["EGO"]["controllers"]["LowLevelController"]
    assert len(llc["calibration"]["steering_map"]) == 21
    assert llc["calibration"]["torque_map"] == "utv_canam_r322_lift_wheel_torque"
    assert llc["roll_filter"]["mode"] == "shadow"
    assert llc["environment"]["vehicle_export"] == "utv_wild_x3_easylift_twin"
    autonomous = llc["calibration"]["autonomous_rate_limits"]
    assert autonomous["steering_rate_max"] > 0
    assert autonomous["torque_rate_max"] > 0
    baseline = dict(llc["roll_filter"])
    baseline.pop("mode")
    for mode in ("active", "shadow"):
        selected = compose_scenario("single_vehicle.yaml", {
            "vehicle": "utv_canam_x3_r322_easylift",
            "preset": f"roll_filter_{mode}.yaml",
        })["vehicles"]["EGO"]["controllers"]["LowLevelController"]
        roll = dict(selected["roll_filter"])
        assert roll.pop("mode") == mode
        assert roll == baseline
        assert selected["calibration"]["autonomous_rate_limits"] == autonomous



@pytest.mark.parametrize("api", ["occupancy_rail", "legacy", "unknown", "", False, None])
def test_torque_map_rejects_removed_selector(api, tmp_path):
    import yaml
    from bng_simulator.utils.scenario_compose import _catalog_entry

    with pytest.raises(ValueError, match="torque_map_api was removed"):
        _catalog_entry({"catalog": {"test": {"torque_map_api": api}}}, "test")
    run = tmp_path / "bad_api.yaml"
    run.write_text(yaml.safe_dump({
        "compose": {"vehicle": "covet_dunes", "level": "tech_ground", "spawn": "origin"},
        "overrides": {"vehicles": {"EGO": {"controllers": {"LowLevelController": {
            "calibration": {"torque_map_api": api}
        }}}}},
    }))
    with pytest.raises(ValueError, match="torque_map_api"):
        compose_scenario(str(run))


def test_torque_map_selection_and_removal():
    from bng_simulator.utils.scenario_compose import _apply_torque_map_rules

    cfg = {}
    _apply_torque_map_rules(cfg, "test_map")
    assert cfg["controllers"]["LowLevelController"]["calibration"] == {"torque_map": "test_map"}
    assert cfg["sensors"]["gtstate"]["torque_map"]["field_name"] == "rear_wheel_torque_est"
    _apply_torque_map_rules(cfg, None)
    assert "torque_map" not in cfg["sensors"]["gtstate"]
    assert "torque_map_api" not in cfg["controllers"]["LowLevelController"]["calibration"]


def test_lua_torque_map_rejects_removed_api_and_invalid_radius(tmp_path):
    import subprocess

    root = Path(__file__).resolve().parents[4]
    script = tmp_path / "torque_api.lua"
    script.write_text("""
package.path = arg[1] .. '/luamod/lua/?.lua;' .. package.path
local message
log=function(_,_,text) message=text end
local llc = require('vehicle.controller.xlab.controller_llc')
for _,api in ipairs({'occupancy_rail','legacy','unknown','',false}) do
  assert(llc.calibrate({torque_map_api=api}) == false)
  assert(message:find('torque_map_api was removed'), message)
end
vec3=function() return {} end
wheels = {wheelRotators = {}, wheelRotatorIDs = {}}
local gt=dofile(arg[1] .. '/luamod/lua/vehicle/controller/xlab/gtState.lua')
log=function() end
for _,r in ipairs({0,-1,0/0,math.huge}) do
  assert(not pcall(gt.setTorqueMapLib,{drivetrain_r_kin=r}))
end
gt.setTorqueMapLib({drivetrain_r_kin=.38})
gt.setTorqueMapLib(nil)
print('PASS: direct Lua calibration rejects removed APIs; GTState validates map radius')
""")
    subprocess.run(["texluajit", "--luaonly", str(script), str(root)], check=True)


@pytest.mark.parametrize("level,failure", [
    (None, None), ("tech_ground", None), (None, "query"), ("tech_ground", "stop"),
])
def test_scenario_stop_checks_loaded_level_and_propagates_failures(monkeypatch, level, failure):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from bng_simulator.core.scenario_builder import ScenarioBuilder
    from bng_simulator.core.xlab_api import XlabApi

    query = Mock(return_value=level)
    if failure == "query":
        query.side_effect = RuntimeError("level query failed")
    monkeypatch.setattr(XlabApi, "get_current_level", query)
    beamng = Mock()
    ack = beamng.scenario._send.return_value.ack
    if failure == "stop":
        ack.side_effect = RuntimeError("scenario stop failed")
    logger = Mock()
    builder = ScenarioBuilder(SimpleNamespace(beamng=beamng, logger=logger))
    if failure:
        with pytest.raises(RuntimeError, match="failed"):
            builder.close_existing_scenario_if_any()
        assert not any("No existing scenario" in str(call) for call in logger.info.call_args_list)
    else:
        builder.close_existing_scenario_if_any()
        if level:
            beamng.scenario._send.assert_called_once_with({"type": "StopScenario"})
            ack.assert_called_once_with("ScenarioStopped")
        else:
            beamng.scenario._send.assert_not_called()


def test_simulator_shutdown_ignores_forwarded_signals(monkeypatch):
    """A process-group SIGINT and launch's forwarded signal share one cleanup."""
    import signal
    from unittest.mock import Mock
    from bng_simulator import sim_manager_node as module

    handlers = {}
    def install(sig, handler):
        previous = handlers.get(sig, signal.SIG_DFL)
        handlers[sig] = handler
        return previous
    monkeypatch.setattr(signal, "signal", install)
    monkeypatch.setattr(module.rclpy, "init", Mock())
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    shutdown = Mock()
    monkeypatch.setattr(module.rclpy, "try_shutdown", shutdown)
    def interrupt(*args, **kwargs):
        handlers[signal.SIGINT](signal.SIGINT, None)
    monkeypatch.setattr(module.rclpy, "spin_once", interrupt)
    node = Mock()
    def cleanup():
        handlers[signal.SIGINT](signal.SIGINT, None)
        handlers[signal.SIGTERM](signal.SIGTERM, None)
    node.destroy_node.side_effect = cleanup
    monkeypatch.setattr(module, "SimulationManagerNode", lambda: node)
    module.main()
    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()
    assert handlers == {signal.SIGINT: signal.SIG_DFL, signal.SIGTERM: signal.SIG_DFL}


def test_llc_torque_map_is_fixed_after_initialization(tmp_path):
    import subprocess

    root = Path(__file__).resolve().parents[4]
    script = tmp_path / "torque_map_lifecycle.lua"
    script.write_text("""
package.path = arg[1] .. '/luamod/lua/?.lua;' .. package.path
log = function() end
local ffi = require('ffi')
local nativeLoads = 0
ffi.load = function()
  nativeLoads = nativeLoads + 1
  return {drivetrain_r_kin=.38,
    drivetrain_forward_torque=function() return 100 end,
    drivetrain_inverse_throttle=function() return .5 end}
end
for _, configured in ipairs({true, false}) do
  local llc = dofile(arg[1] .. '/luamod/lua/vehicle/controller/xlab/controller_llc.lua')
  local map = {{u=-1,delta=-.5},{u=1,delta=.5}}
  local initial = {steering_map=map, gains={kp=10}}
  if configured then initial.torque_map='old_map' end
  assert(llc.calibrate(initial))
  local gt = {registerCustomField=function() end, setTorqueMapLib=function() end}
  local common = {gtStateManager={getGtStateController=function() return gt end}, gtStateSensorId=1}
  if configured then common.torqueMapPath='mock_library' end
  assert(llc.init(common))
  local loads = nativeLoads
  if configured then
    assert(llc.onTune({calibration={torque_map='old_map', gains={kp=20}}}))
    assert(llc.getStatus().gains.kp == 20)
  end
  local before = llc.getStatus()
  assert(llc.onTune({calibration={torque_map='new_map', gains={kp=30}}}) == false)
  local after = llc.getStatus()
  assert(after.torqueMapStem == before.torqueMapStem and after.gains.kp == before.gains.kp)
  assert(nativeLoads == loads)
  llc.reset()
  assert(llc.calibrate({torque_map='new_map'}) == false)
  if configured then assert(llc.calibrate({torque_map='old_map'})) end
  llc.stop()
  assert(llc.calibrate({torque_map='new_map'}))
end
print('PASS torque-map startup, same-name tune, rejected atomic change, reset, stop and no-map startup')
""")
    subprocess.run(["texluajit", "--luaonly", str(script), str(root)], check=True)


def test_ge_native_library_cache_and_staging_failures(tmp_path):
    import subprocess

    root = Path(__file__).resolve().parents[4]
    script = tmp_path / "native_staging.lua"
    script.write_text("""
log = function() end
extensions = {scenario_scenariosLoader=true}
Engine = {Sandbox={Lua={isEnabled=function() return false end}}}
scenetree = {findObject=function() return {getID=function() return 1 end} end}
local files, manifests = {}, {}
local copies, sent, encoded = 0, 0, nil
local failCopy, badPath = false, false
readFile = function(path) return files[path] end
jsonReadFile = function(path) return manifests[path] end
copyfile = function(source, target)
  copies = copies + 1
  if failCopy then return false end
  files[target] = files[source]
  return true
end
FS = {virtual2Native=function(_, path) return not badPath and ('native/'..path) or nil end}
lpack = {encode=function(data) encoded=data; return 'payload' end}
be = {queueObjectLua=function() sent=sent+1 end}
local ge = dofile(arg[1] .. '/luamod/lua/ge/extensions/xlab/xlabCore.lua')
local function open(extra)
  local request = {name='EGO',vid='EGO',controllerType='llc',sendACK=function() end}
  for k,v in pairs(extra) do request[k]=v end
  ge.handleOpenController(request)
end
for _, windows in ipairs({false,true}) do
  jit.os = windows and 'Windows' or 'Linux'
  files, manifests = {}, {}
  local extension = windows and '.dll' or '.so'
  local target = windows and 'windows' or 'linux'
  local base = 'lua/vehicle/controller/xlab/lib/'
  local manifest = {target=target, library='test_map'..extension, library_sha256=string.rep('a',64)}
  manifests[base..'test_map.manifest.json'] = manifest
  files[base..manifest.library] = 'compiled map A'
  open({calibration={torque_map='test_map'}})
  local first = encoded.torqueMapPath
  local copied = copies
  failCopy = true -- a loaded DLL cannot be overwritten; a cache hit needs no copy
  open({calibration={torque_map='test_map'}})
  assert(encoded.torqueMapPath == first and copies == copied)
  manifest.library_sha256 = string.rep('b',64)
  files[base..manifest.library] = 'compiled map B'
  local sentBefore = sent
  assert(not pcall(open,{calibration={torque_map='test_map'}}))
  assert(sent == sentBefore)
  failCopy = false
  open({calibration={torque_map='test_map'}})
  assert(encoded.torqueMapPath ~= first)
  local cached = 'tmp/xlab_'..manifest.library_sha256..extension
  files[cached] = 'corrupt cache'
  assert(not pcall(open,{calibration={torque_map='test_map'}}))
  files[cached] = 'compiled map B'
  badPath = true
  assert(not pcall(open,{calibration={torque_map='test_map'}}))
  badPath = false
  files[base..manifest.library] = nil
  assert(not pcall(open,{calibration={torque_map='test_map'}}))
  assert(not pcall(open,{calibration={torque_map='../invalid'}}))
  files[base..manifest.library] = 'compiled map B'
  manifest.target = 'unsupported'
  assert(not pcall(open,{calibration={torque_map='test_map'}}))
  manifest.target = target
  manifests[base..'test_map.manifest.json'] = nil
  assert(not pcall(open,{calibration={torque_map='test_map'}}))
  manifests[base..'test_map.manifest.json'] = manifest
  manifest.library_sha256 = 'bad hash'
  assert(not pcall(open,{calibration={torque_map='test_map'}}))
  -- Native vehicle exports use exactly the same verified cache path.
  local envBase = base..'vehicles/test_vehicle/'
  local envManifest = {target=target, precision='float32',environment_abi_version=1,
    library=windows and 'roll_filter.dll' or 'libroll_filter.so',library_sha256=string.rep('c',64)}
  manifests[envBase..'manifest.json'] = envManifest
  files[envBase..envManifest.library] = 'compiled environment'
  open({environment={vehicle_export='test_vehicle'}})
  assert(encoded.environment.library_path == 'native/tmp/xlab_'..string.rep('c',64)..extension)
  copied = copies
  failCopy = true
  open({environment={vehicle_export='test_vehicle'}})
  assert(copies == copied)
  failCopy = false
end
print('PASS Linux/Windows immutable cache, changed bytes, copy failure, corrupt cache and invalid inputs')
""")
    subprocess.run(["texluajit", "--luaonly", str(script), str(root)], check=True)
