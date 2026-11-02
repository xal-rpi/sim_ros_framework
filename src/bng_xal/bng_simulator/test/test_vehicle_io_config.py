"""Vehicle I/O configuration, calibration commands and garage session ownership."""

from __future__ import annotations

import pytest

from bng_simulator.utils.scenario_compose import compose_scenario
from bng_simulator.utils.vehicle_io_config import (
    inject_vehicle_io_ports,
    resolved_udp_ports,
)


def _sample_scenario() -> dict:
    return compose_scenario("multi_agent.yaml")


def test_resolved_udp_ports_stride() -> None:
    scenario = _sample_scenario()
    ego_cfg = scenario["vehicles"]["EGO"]
    agent2_cfg = scenario["vehicles"]["AGENT2"]
    base_listen, base_send, base_sensor = resolved_udp_ports(
        scenario,
        ego_cfg,
        listen_port=64257,
        send_port=64258,
        sensor_port=64259,
    )
    agent2_ports = resolved_udp_ports(
        scenario,
        agent2_cfg,
        listen_port=64257,
        send_port=64258,
        sensor_port=64259,
    )
    stride = scenario["udp_io"]["port_stride"]
    assert agent2_ports[0] == base_listen + stride
    assert agent2_ports[1] == base_send + stride
    assert agent2_ports[2] == base_sensor + stride


def test_inject_vehicle_io_ports_into_llc() -> None:
    scenario = _sample_scenario()
    inject_vehicle_io_ports(
        scenario,
        beamng_host="10.0.0.1",
        remote_host="10.0.0.2",
        listen_port=64000,
        send_port=64001,
        sensor_port=64002,
    )
    ego_llc = scenario["vehicles"]["EGO"]["controllers"]["LowLevelController"]
    agent2_llc = scenario["vehicles"]["AGENT2"]["controllers"]["LowLevelController"]
    assert ego_llc["control_listen"] == 64000
    assert ego_llc["control_listen_ip"] == "10.0.0.1"
    assert ego_llc["sensor_send_ip"] == "10.0.0.2"
    assert agent2_llc["control_listen"] == 64010


def test_gear_scan_preserves_initial_neutral_not_revisited(monkeypatch):
    from bng_simulator.scripts import sim_control

    current = {"index": 0}
    commands = []

    def request(name, args, **kwargs):
        if name == "set_gearbox_index":
            commands.append(args["gear_index"])
            # The observed scan never returns to its initial neutral state.
            current["index"] = -1 if args["gear_index"] < 1 else 1
            return {}
        assert name == "get_gearbox_info"
        index = current["index"]
        return {
            "minGearIndex": -1, "maxGearIndex": 1,
            "gearName": float(index), "gearIndex": index,
            "gearModeIndex": 0, "gearRatio": index * 3.5, "mode": "",
        }

    monkeypatch.setattr(sim_control, "send_request", request)
    monkeypatch.setattr(sim_control.time, "sleep", lambda _: None)
    gears = sim_control.SimulationController().extract_gear_infos("EGO")
    assert gears[0]["gear_ratio"] == 0
    assert gears[0]["indexCmd"] == 0
    assert set(gears) == {-1, 0, 1}
    assert commands[-1] == 0


@pytest.fixture
def garage_case():
    from copy import deepcopy

    node = {"cid": 3, "name": "body", "group": "frame", "chassisFLU": [0, 0, 0]}
    inventory = {
        "nodes": [node], "referenceNodes": {"ref": 0, "back": 1, "up": 2},
        "gtControllers": [{"name": "gt", "contactEnabled": True}], "status": {"active": False},
        "vehicle": {"name": "EGO", "model": "test", "partConfig": "test.pc"},
    }
    plan = {
        "schemaVersion": 1, "referenceNodes": inventory["referenceNodes"],
        "selectedNodeReference": [node], "vehicle": inventory["vehicle"],
        "config": {"cogHeightEstimate": 0.6, "gtController": "gt", "nodes": {"seat": [3]},
                   "testPlacements": [{"name": "seat", "weights": {"seat": 1}}]},
    }
    return deepcopy(inventory), deepcopy(plan)


@pytest.mark.parametrize("change", [
    "schema", "unknown_plan", "unknown_config", "null_height", "boolean_mass",
    "negative_age", "fractional_levels", "duplicate_node", "unknown_node_option",
    "unknown_placement", "unknown_location", "bad_weight_sum", "empty_placements",
    "wrong_vehicle", "changed_geometry", "unknown_direction", "wrong_reference",
])
def test_garage_rejects_invalid_plans_before_start(garage_case, change, tmp_path):
    from bng_simulator.core import garage_control as garage

    inventory, plan = garage_case
    cfg = plan["config"]
    if change == "schema":
        plan["schemaVersion"] = 999
    elif change == "unknown_plan":
        plan["typo"] = 1
    elif change == "unknown_config":
        cfg["ballastkg"] = 10
    elif change == "null_height":
        cfg["cogHeightEstimate"] = None
    elif change == "boolean_mass":
        cfg["ballastKg"] = True
    elif change == "negative_age":
        cfg["maxLoadAge"] = -0.1
    elif change == "fractional_levels":
        cfg["levels"] = 1.5
    elif change == "duplicate_node":
        cfg["nodes"]["seat"] = [3, 3]
    elif change == "unknown_node_option":
        cfg["nodes"]["seat"] = [{"cid": 3, "weight": 1, "typo": True}]
    elif change == "unknown_placement":
        cfg["testPlacements"][0]["typo"] = 1
    elif change == "unknown_location":
        cfg["testPlacements"][0]["weights"] = {"missing": 1}
    elif change == "bad_weight_sum":
        cfg["testPlacements"][0]["weights"] = {"seat": 0.5}
    elif change == "empty_placements":
        cfg["testPlacements"] = []
    elif change == "wrong_vehicle":
        plan["vehicle"]["model"] = "other"
    elif change == "changed_geometry":
        inventory["nodes"][0]["chassisFLU"] = [1, 0, 0]
    elif change == "unknown_direction":
        cfg["forceDirection"] = "sideways"
    else:
        cfg["referenceNodes"] = {"ref": 5, "back": 1, "up": 2}
    actions = []

    def send(name, args):
        actions.append(args["action"])
        assert args["action"] == "inspect"
        return inventory

    with pytest.raises(ValueError):
        garage.run(send, plan, tmp_path / "run")
    assert actions == ["inspect"]
    assert not (tmp_path / "run").exists()


@pytest.mark.parametrize("failure", [None, "status_identity", "read_identity", "stop_identity", "start_timeout", "recording", "interrupt"])
def test_garage_run_owns_session_and_preserves_recoverable_data(garage_case, tmp_path, monkeypatch, failure):
    import json
    from bng_simulator.core import garage_control as garage

    inventory, plan = garage_case
    monkeypatch.delenv("BEAMNG_USER_PATH", raising=False)
    actions, expected, status_calls = [], None, 0
    status = {"active": True, "phase": "reference", "elapsed": 0}

    def send(name, args):
        nonlocal expected, status_calls
        assert name == "garage_experiment"
        action = args["action"]
        actions.append(action)
        if action == "inspect":
            return inventory
        if action == "start":
            expected = args["config"]["output"]
            assert args["expected_output"] == expected
            if failure == "start_timeout":
                raise TimeoutError("lost start reply")
            return {"output": expected, "duration": 1}
        assert args["expected_output"] == expected
        if action == "status":
            status_calls += 1
            if failure == "status_identity":
                return {"active": True, "output": "other"}
            if failure in ("stop_identity", "interrupt"):
                if failure == "interrupt" and status_calls == 1:
                    raise KeyboardInterrupt("operator stop")
                return {**status, "output": expected}
            if failure == "start_timeout":
                return {**status, "output": expected}
            return {"active": False, "reason": "cleanup error" if failure == "recording" else "complete",
                    "recordingErrors": ["write failed"] if failure == "recording" else [], "output": expected}
        if action == "stop":
            if failure == "stop_identity":
                # The Lua-side identity guard rejects a replacement before restoring it.
                raise RuntimeError("session identity changed")
            status.update(active=False, reason="CLI stop")
            return {**status, "output": expected}
        assert action == "read"
        return {"output": "other" if failure == "read_identity" else expected,
                "data": '{}\n', "nextOffset": 3, "eof": True}

    output = tmp_path / "run"
    # The timeout forces the active stop_identity case into cleanup promptly.
    ticks = iter(range(100))
    kwargs = {"clock": lambda: next(ticks), "sleep": lambda _: None, "max_wall_seconds": 0.5}
    if failure is None:
        result = garage.run(send, plan, output, **kwargs)
        assert result["status"]["reason"] == "complete"
    else:
        with pytest.raises((RuntimeError, TimeoutError, KeyboardInterrupt)):
            garage.run(send, plan, output, **kwargs)
    finished = json.loads((output / "finished.json").read_text())
    assert bool(finished["error"]) == (failure is not None)
    assert json.loads((output / "started.json").read_text())["output"] == expected
    if failure in ("status_identity", "stop_identity", "read_identity"):
        assert not (output / "recording.jsonl").exists()
    else:
        assert (output / "recording.jsonl").read_bytes() == b'{}\n'
    if failure == "status_identity":
        assert "stop" not in actions and "read" not in actions
    if failure == "stop_identity":
        assert "read" not in actions
    if failure in ("start_timeout", "interrupt"):
        assert "stop" in actions and "read" in actions
    assert sorted(p.name for p in output.iterdir()) == sorted(
        ["inventory.json", "plan.yaml", "started.json", "finished.json"]
        + (["recording.jsonl"] if (output / "recording.jsonl").exists() else [])
    )


@pytest.mark.parametrize("failure", [None, "wrong_identity", "stalled", "wrong_offset", "read_error"])
def test_garage_transfer_is_atomic_and_checks_every_chunk(tmp_path, monkeypatch, failure):
    from bng_simulator.core import garage_control as garage

    monkeypatch.delenv("BEAMNG_USER_PATH", raising=False)
    status = {"active": False, "output": "/settings/support_garage_owned.jsonl"}
    calls = 0

    def send(name, args):
        nonlocal calls
        calls += 1
        assert args["expected_output"] == status["output"]
        if calls == 1:
            return {"output": status["output"], "data": 'a\n', "nextOffset": 2, "eof": False}
        if failure == "read_error":
            raise OSError("transfer failed")
        return {"output": "other" if failure == "wrong_identity" else status["output"],
                "data": "" if failure == "stalled" else 'b\n',
                "nextOffset": 2 if failure in ("stalled", "wrong_offset") else 4,
                "eof": failure != "stalled"}

    destination = tmp_path / "recording.jsonl"
    if failure:
        with pytest.raises((ValueError, RuntimeError, OSError)):
            garage.download_recording(send, status, destination)
        assert not list(tmp_path.iterdir())
    else:
        garage.download_recording(send, status, destination)
        assert destination.read_bytes() == b'a\nb\n'
        with pytest.raises(FileExistsError):
            garage.download_recording(send, status, destination)


def test_garage_direct_copy_uses_only_explicit_user_path(tmp_path, monkeypatch):
    from bng_simulator.core import garage_control as garage

    status = {"active": False, "output": "/settings/support_garage_owned.jsonl"}
    destination = tmp_path / "recording.jsonl"
    monkeypatch.delenv("BEAMNG_USER_PATH", raising=False)
    assert garage.copy_local_recording(status, destination) is False
    root = tmp_path / "game"
    monkeypatch.setenv("BEAMNG_USER_PATH", str(root))
    with pytest.raises(FileNotFoundError):
        garage.copy_local_recording(status, destination)
    source = root / "settings/support_garage_owned.jsonl"
    source.parent.mkdir(parents=True)
    source.write_bytes(b'{}\n')
    assert garage.copy_local_recording(status, destination)
    assert destination.read_bytes() == source.read_bytes()
    assert sorted(p.name for p in tmp_path.iterdir()) == ["game", "recording.jsonl"]


def test_garage_placement_mapping_and_canonical_profile(garage_case, monkeypatch):
    from pathlib import Path
    import yaml
    from bng_simulator.core import garage_control as garage
    from bng_simulator.utils import scenario_compose

    inventory, _ = garage_case
    profile = {"seat": {"position_chassis_flu_m": [0, 0, 0], "node_pattern": "frame", "count": 1}}
    assert garage.select_nodes(inventory, profile) == {"seat": [3]}
    for extra in ({"typo": 1}, {"count": 1.5}, {"radius_m": float("nan")}, {"node_names": ["body"]}):
        with pytest.raises(ValueError):
            garage.select_nodes(inventory, {"seat": {**profile["seat"], **extra}})
    config_dir = Path(__file__).resolve().parents[2] / "bng_bringup/config"
    monkeypatch.setattr(scenario_compose, "bringup_config_dir", lambda: str(config_dir))
    inventory["vehicle"]["model"] = "utv"
    inventory["referenceNodes"]["partOrigin"] = "canam_x3_frame_base_808"
    canonical = garage.utv_seat_profile(inventory)
    inventory["nodes"] = [
        {"cid": i, "name": name, "group": "utv_frame", "chassisFLU": [0, 0, 0]}
        for i, name in enumerate(n for spec in canonical["placements"].values() for n in spec["node_names"])
    ]
    plan = garage.make_plan(inventory, canonical)
    with pytest.raises(ValueError, match="cogHeightEstimate"):
        garage.validate_plan(plan, inventory)
    plan["config"]["cogHeightEstimate"] = 0.6
    resolved = garage.validate_plan(yaml.safe_load(yaml.safe_dump(plan)), inventory)
    assert resolved["levels"] == 1 and len(resolved["nodes"]) == 3


def test_garage_recovery_rechecks_identity_and_allows_partial_data(tmp_path, monkeypatch):
    import json
    from bng_simulator.core import garage_control as garage

    monkeypatch.delenv("BEAMNG_USER_PATH", raising=False)
    expected = "/settings/support_garage_owned.jsonl"
    (tmp_path / "started.json").write_text(json.dumps({"output": expected}))
    actions = []

    def send(name, args):
        actions.append(args["action"])
        assert args["expected_output"] == expected
        if args["action"] == "status":
            return {"active": True, "output": expected}
        if args["action"] == "stop":
            return {"active": False, "reason": "cleanup error", "recordingErrors": ["flush failed"], "output": expected}
        return {"data": '{}\n', "nextOffset": 3, "eof": True, "output": expected}

    with pytest.raises(ValueError, match="still active"):
        garage.recover(send, tmp_path)
    result = garage.recover(send, tmp_path, stop_active=True)
    assert result["status"]["recordingErrors"]
    assert (tmp_path / "recording.recovered.jsonl").read_bytes() == b'{}\n'
    assert actions.count("stop") == 1


def test_garage_request_relay_forwards_session_identity():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from bng_simulator.core.request_handler import SimulationRequestHandler

    vehicle = Mock()
    vehicle._root._send.return_value.recv.return_value = {"data": {"active": False, "output": "owned"}}
    sim = SimpleNamespace(logger=Mock(), default_vehicle_name="EGO",
                          vehicles={"EGO": SimpleNamespace(vehicle=vehicle)})
    handler = SimulationRequestHandler(sim)
    result = handler.garage_experiment("read", expected_output="owned", offset=10, limit=20)
    assert result["output"] == "owned"
    vehicle._root._send.assert_called_once_with({
        "type": "SupportGarage", "action": "read", "expectedOutput": "owned", "offset": 10, "limit": 20,
    })


def test_lua_garage_validation_identity_and_cleanup_failures(tmp_path):
    import subprocess
    from pathlib import Path

    root = Path(__file__).resolve().parents[4]
    script = tmp_path / "garage_failures.lua"
    script.write_text("""
local path = arg[1] .. '/luamod/lua/vehicle/extensions/xlab/supportGarage.lua'
local function copy(value)
  if type(value) ~= 'table' then return value end
  local result = {}
  for key, item in pairs(value) do result[key] = copy(item) end
  return result
end
local config = {output='/settings/support_garage_owned.jsonl', gtController='gt',
  cogHeightEstimate=.6, nodes={seat={3}}, testPlacements={{name='seat',weights={seat=1}}}}
local garage = dofile(path)
local resolved = garage.validateConfig(config)
assert(config.levels == nil and resolved.levels == 1 and resolved.ballastKg == 10)
local rounded = copy(config)
rounded.nodes.rear = {4}
rounded.testPlacements[1].weights = {seat=0.4000000059604645,rear=0.6000000238418579}
assert(garage.validateConfig(rounded))
local bad = {
  function(c) c.ballastkg=1 end,
  function(c) c.cogHeightEstimate=nil end,
  function(c) c.forceN=false end,
  function(c) c.maxLoadAge=1 end,
  function(c) c.nodes.seat={3,3} end,
  function(c) c.nodes.seat={{cid=3,weight=1,typo=1}} end,
  function(c) c.testPlacements[1].typo=true end,
  function(c) c.testPlacements[1].weights={missing=1} end,
  function(c) c.testPlacements[1].forceDirection='up' end,
  function(c) c.testPlacements={} end,
  function(c) c.levels=1.5 end,
}
-- No engine or filesystem API exists yet: rejection must precede side effects.
for _, change in ipairs(bad) do
  local candidate=copy(config)
  change(candidate)
  assert(not pcall(garage.validateConfig,candidate))
  assert(not pcall(garage.start,candidate))
end
local function installState(module, value)
  for index=1,50 do
    local name=debug.getupvalue(module.stop,index)
    if not name then break end
    if name=='state' then debug.setupvalue(module.stop,index,value);return end
  end
  error('garage state upvalue missing')
end
jsonEncode = function() return '{}' end
for _, failure in ipairs({'none','write','flush','close','restore','all'}) do
  garage=dofile(path)
  local restored, closed, writes = {}, false, 0
  obj={getSimTime=function() return 1 end,
    setNodeMass=function(_,id,mass)
      restored[id]=mass
      if (failure=='restore' or failure=='all') and id==3 then error('restore failed') end
    end}
  local file={
    write=function(self)
      writes=writes+1
      if failure=='write' or failure=='all' then return nil,'disk full' end
      return self
    end,
    flush=function()
      if failure=='flush' or failure=='all' then return nil,'flush failed' end
      return true
    end,
    close=function()
      closed=true
      if failure=='close' or failure=='all' then return nil,'close failed' end
      return true
    end,
  }
  installState(garage,{baselineMass={[3]=10,[4]=20}, config=config, file=file,
    schedule={{name='reference'}},phase=1,elapsed=0})
  assert(not pcall(garage.status,'other'))
  assert(not pcall(garage.stop,'complete','other'))
  assert(not pcall(garage.readRecording,0,10,'other'))
  assert(next(restored)==nil and writes==0 and not closed)
  local ok=pcall(garage.stop,'complete',config.output)
  assert(ok == (failure=='none'))
  assert(restored[3]==10 and restored[4]==20 and closed)
  local status=garage.status(config.output)
  assert(status.active==false)
  assert((status.reason=='complete') == (failure=='none'))
  assert((#status.restoreErrors>0) == (failure=='restore' or failure=='all'))
  assert((#status.recordingErrors>0) == (failure=='write' or failure=='flush' or failure=='close' or failure=='all'))
  garage.stop('idempotent',config.output)
  assert(not pcall(garage.readRecording,0,10,'other'))
end
-- The vehicle request router must preserve the identity all the way to Lua.
local observed={}
extensions={xlab_supportGarage={
  status=function(expected) observed.status=expected;return {active=false,output=expected} end,
  stop=function(reason,expected) observed.stop=expected end,
  readRecording=function(offset,limit,expected) observed.read={offset,limit,expected};return {output=expected} end,
}}
local router=dofile(arg[1]..'/luamod/lua/vehicle/extensions/xlab/xlabCore.lua')
for _,action in ipairs({'status','stop','read'}) do
  router.handleSupportGarage({action=action,expectedOutput='owned',offset=2,limit=4,
    sendResponse=function(_,result) assert(not result.data.error) end})
end
assert(observed.status=='owned' and observed.stop=='owned')
assert(observed.read[1]==2 and observed.read[2]==4 and observed.read[3]=='owned')
print('PASS garage validation, atomic session guards, unconditional restoration and recording/restore failures')
""")
    subprocess.run(["texluajit", "--luaonly", str(script), str(root)], check=True)


@pytest.mark.parametrize("owned", [True, False])
def test_garage_service_client_cleans_failed_entry_without_shutting_shared_context(monkeypatch, owned):
    import sys
    from types import SimpleNamespace
    from unittest.mock import Mock
    from bng_simulator.core import garage_control as garage

    live = {"ok": not owned}
    node, client = Mock(), Mock()
    node.create_client.return_value = client
    client.wait_for_service.return_value = False
    ros = SimpleNamespace(
        ok=lambda: live["ok"],
        init=Mock(side_effect=lambda: live.update(ok=True)),
        shutdown=Mock(), create_node=Mock(return_value=node),
    )
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    with pytest.raises(TimeoutError, match="service unavailable"):
        with garage.GarageServiceClient():
            pytest.fail("missing service cannot enter a session")
    node.destroy_node.assert_called_once()
    assert ros.init.call_count == int(owned)
    assert ros.shutdown.call_count == int(owned)
