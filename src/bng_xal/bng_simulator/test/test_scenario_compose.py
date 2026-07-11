"""Unit tests for scenario compose / preset / launch override resolution."""

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


def test_resolve_config_path_gridworld() -> None:
    path = resolve_config_path("gridworld.yaml")
    assert path.endswith("runs/gridworld.yaml")


def test_compose_gridworld_defaults() -> None:
    cfg = compose_scenario("gridworld.yaml")
    ego = cfg["vehicles"]["EGO"]
    assert cfg["scenario"]["level"] == "tech_ground"
    assert ego["spawn"]["pos"] == [0, 0, 0]
    assert ego["spawn"]["xlab_yaw_deg"] == 0
    assert ego["io"]["port_index"] == 0


def test_compose_preset_overrides_level_and_spawn() -> None:
    cfg = compose_scenario("gridworld.yaml", {"preset": "derby_grid_lane.yaml"})
    ego = cfg["vehicles"]["EGO"]
    assert cfg["scenario"]["level"] == "derby"
    assert ego["spawn"]["pos"] == pytest.approx([-0.243, 148.0445, 79.3])


def test_compose_preset_then_launch_yaw_wins(sample_preset: Path) -> None:
    cfg = compose_scenario(
        "gridworld.yaml",
        {"preset": sample_preset.name, "yaw": 45},
    )
    assert cfg["vehicles"]["EGO"]["spawn"]["xlab_yaw_deg"] == 45.0


def test_compose_launch_pos_override() -> None:
    cfg = compose_scenario(
        "gridworld.yaml",
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


def test_compose_yaw_offset_utv() -> None:
    cfg = compose_scenario("gridworld.yaml")
    spawn = cfg["vehicles"]["EGO"]["spawn"]
    # utv catalog yaw_offset_deg=90 → beamng spawn yaw = xlab - offset
    assert spawn["yaw_angle"] == beamng_yaw_from_xlab(0.0, 90.0)


def test_unknown_spawn_raises() -> None:
    with pytest.raises(KeyError, match="Unknown spawn preset"):
        compose_scenario("gridworld.yaml", {"spawn": "does_not_exist"})


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
        textwrap.dedent(
            """\
            level: derby
            spawn: grid_lane
            pos: [10.0, 20.0, 30.0]
            """
        ),
        encoding="utf-8",
    )
    cfg = compose_scenario("gridworld.yaml", {"preset": "abs_pos.yaml"})
    assert cfg["vehicles"]["EGO"]["spawn"]["pos"] == [10.0, 20.0, 30.0]


def test_summarize_config_gridworld() -> None:
    cfg = compose_scenario("gridworld.yaml")
    summary = summarize_config(cfg, config_path="gridworld.yaml")
    assert "=== xlab scenario manifest ===" in summary
    assert "vehicle EGO" in summary
    assert "tech_ground" in summary
    assert "/EGO/control/cmd" in summary
    assert "control_state@" in summary


def test_summarize_config_launch_overrides() -> None:
    cfg = compose_scenario("gridworld.yaml", {"yaw": 45, "level": "derby"})
    summary = summarize_config(
        cfg,
        config_path="gridworld.yaml",
        launch_overrides={"yaw": 45, "level": "derby"},
    )
    assert "yaw=45" in summary
    assert "level=derby" in summary
