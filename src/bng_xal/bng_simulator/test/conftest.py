"""Shared fixtures for bng_simulator unit tests."""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest


@pytest.fixture
def tmp_preset_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Redirect user preset search to an isolated directory."""
    preset_dir = tmp_path / "presets" / "user"
    preset_dir.mkdir(parents=True)

    import bng_simulator.utils.scenario_compose as sc

    monkeypatch.setattr(sc, "user_preset_search_dirs", lambda: [str(preset_dir)])
    return preset_dir


@pytest.fixture
def sample_preset(tmp_preset_dir: Path) -> Path:
    path = tmp_preset_dir / "test_derby.yaml"
    path.write_text(
        textwrap.dedent(
            """\
            level: derby
            spawn: grid_lane
            yaw: 10
            """
        ),
        encoding="utf-8",
    )
    return path
