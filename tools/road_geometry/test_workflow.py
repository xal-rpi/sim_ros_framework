"""Saved-route publication, editor recipes and downstream observation contracts."""

import copy
import json
import numpy as np
import pytest
import subprocess
from local_server import save_document, selection_path
from pathlib import Path
from reference_path import replay_reference
from route_context import nearby
from route_schema import VERSION, normalize_arrays, normalize_export, validate_routes
from route_selection import (
    digest,
    load_route,
    materialize,
    reorder,
    selection_rows,
    validate_recipe,
)
from track_recipe import corridor_queries, save_track


def observations():
    center = np.array([[0.0, 0, 3], [2.0, 0, 4], [4.0, 0, 3]])
    # Banked sections: 2 m along the profile is only sqrt(3) m horizontally.
    xyz = (
        center[:, None, :] + [[0, -np.sqrt(3), -1], [0, 0, 0], [0, np.sqrt(3), 1]]
    ).reshape(-1, 3)
    return dict(
        station_id=np.array([10, 11, 12]),
        center_xyz=center,
        reference_xyz=center.copy(),
        section_xyz=xyz,
        section_offsets=np.array([0, 3, 6, 9]),
    )


def test_banked_and_horizontal_acquisitions_share_coordinate_and_limits_meanings():
    base = observations()
    managed = base | dict(
        design_reference_xyz=base["center_xyz"] + [0, 0, 0.2],
        design_forward_axis=np.tile([1.0, 0, 0], (3, 1)),
        section_lateral_m=np.tile([-2.0, 0, 2], 3),
        profile_limits_m=np.tile([-2.0, 2], (3, 1)),
    )
    managed.pop("reference_xyz")
    measured = base | dict(
        reference_xyz=managed["design_reference_xyz"],
        horizontal_left_axis=np.tile([0.0, 1, 0], (3, 1)),
        section_lateral_m=np.tile([-np.sqrt(3), 0, np.sqrt(3)], 3),
        chosen_limits_m=np.tile([-1.0, 1.0], (3, 1)),
    )
    a = normalize_arrays(managed, dict(topology="open"))
    b = normalize_arrays(measured, dict(topology="open"))
    for key in [
        "center_xyz",
        "reference_xyz",
        "section_xyz",
        "section_lateral_xy_m",
        "sampled_limits_xy_m",
        "center_distance_xyz_m",
    ]:
        np.testing.assert_array_equal(a[key], b[key])
    np.testing.assert_allclose(
        a["section_lateral_xy_m"][:3], [-np.sqrt(3), 0, np.sqrt(3)]
    )
    np.testing.assert_array_equal(a["source_section_lateral_m"][:3], [-2, 0, 2])
    assert not a["requested_limits_valid"].any()
    np.testing.assert_array_equal(b["requested_limits_xy_m"][0], [-1, 1])
    for arrays in [a, b]:
        assert not arrays["physical_limits_valid"].any()
        assert not arrays["optimization_limits_valid"].any()
        assert not arrays["section_normal_valid"].any()
        assert np.isnan(arrays["section_normal"]).all()
        assert "section_lateral_m" not in arrays and "chosen_limits_m" not in arrays


def test_reference_and_center_chord_lengths_are_distinct_and_missing_stays_missing():
    a = observations()
    a["reference_xyz"] = a["center_xyz"].copy()
    a["reference_xyz"][:, 2] = 10.0
    a["reference_distance_xyz_m"] = [
        100.0,
        102,
        104,
    ]  # prior authored progress coordinate
    a["section_xyz"][0, 2] = np.nan
    b = normalize_arrays(a, dict(topology="open"))
    np.testing.assert_allclose(b["center_distance_xy_m"], [0, 2, 4])
    np.testing.assert_allclose(
        b["center_distance_xyz_m"], [0, np.sqrt(5), 2 * np.sqrt(5)]
    )
    np.testing.assert_array_equal(b["reference_distance_xyz_m"], [0, 2, 4])
    np.testing.assert_array_equal(b["source_reference_distance_xyz_m"], [100, 102, 104])
    assert not b["section_valid"][0] and b["section_lateral_valid"][0]
    assert np.isnan(b["section_xyz"][0, 2])
    assert (
        b["sampled_limits_xy_m"][0, 0] == 0
    )  # missing right sample isn't a known edge


def test_common_reversal_swaps_limits_but_preserves_world_normals_and_source_projection():
    a = normalize_arrays(
        observations() | dict(horizontal_left_axis=np.tile([0.0, 1, 0], (3, 1))),
        dict(topology="open"),
    )
    a["section_normal"][:] = [0.0, -0.5, np.sqrt(0.75)]
    a["section_normal_valid"][:] = True
    a["requested_limits_xy_m"][:] = [-1, 2]
    a["requested_limits_valid"][:] = True
    a["source_section_lateral_m"] = np.tile([-2.0, 0, 2], 3)
    recipe = dict(start_row=2, end_row=0, direction="reverse", full_loop=False)
    b = reorder(a, dict(topology="open"), recipe)
    np.testing.assert_array_equal(b["section_xyz"][:3], a["section_xyz"][8:5:-1])
    np.testing.assert_array_equal(b["section_normal"], a["section_normal"])
    np.testing.assert_array_equal(b["requested_limits_xy_m"][0], [-2, 1])
    np.testing.assert_array_equal(b["source_section_lateral_m"][:3], [2, 0, -2])
    np.testing.assert_allclose(
        b["section_lateral_xy_m"][:3], [-np.sqrt(3), 0, np.sqrt(3)]
    )
    assert b["center_distance_xyz_m"][0] == 0


def test_migration_keeps_geometry_and_saved_choices_valid_and_new_generation_uses_same_schema(
    tmp_path,
):
    a = observations()
    np.savez_compressed(tmp_path / "route.npz", **a)
    meta = dict(
        schema_version="0.3-track-samples",
        topology="open",
        query_construction="test horizontal sections",
        limits_convention="unknown",
        source={},
        surfaces=[],
        unavailable=[],
        route_sha256=digest(tmp_path / "route.npz"),
    )
    (tmp_path / "metadata.json").write_text(json.dumps(meta))
    recipe = dict(
        schema_version="0.1-route-selection",
        frame="local_ENU",
        name="backwards",
        route_name=tmp_path.name,
        source_metadata_sha256=digest(tmp_path / "metadata.json"),
        source_route_sha256=digest(tmp_path / "route.npz"),
        start_row=2,
        end_row=0,
        direction="reverse",
        full_loop=False,
    )
    m = normalize_export(tmp_path)
    b, _, definition, _ = load_route(tmp_path)
    for name in ["station_id", "center_xyz", "section_xyz", "section_offsets"]:
        np.testing.assert_array_equal(b[name], a[name])
    validate_recipe(tmp_path, recipe)
    assert normalize_export(tmp_path) == m  # idempotent; hashes stay stable
    result = tmp_path / "exports/backwards"
    materialize(tmp_path, recipe, result)
    c, selected_meta, selected_def, _ = load_route(result)
    assert selected_meta["schema_version"] == VERSION
    assert set(definition) == set(selected_def)
    assert set(b) == set(c)
    assert not selected_meta["notation_previous_hashes"]
    assert validate_routes(result)["routes"][0]["array_checks_passed"]
    recipe["source_route_sha256"] = "wrong"
    with pytest.raises(ValueError, match="different source"):
        validate_recipe(tmp_path, recipe)


def source(tmp_path, closed=True):
    centers = np.array([[0.0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 0, 0]])
    if not closed:
        centers = centers[:-1]
    n = len(centers)
    a = dict(
        station_id=np.arange(n) + 10,
        center_xyz=centers,
        reference_xyz=centers + np.array([0, 0, 1]),
        center_valid=np.ones(n, bool),
        horizontal_left_axis=np.tile([0.0, 1, 0], (n, 1)),
        section_xyz=(centers[:, None, :] + np.array([[0, -1, 0], [0, 2, 0]])).reshape(
            -1, 3
        ),
        section_offsets=np.arange(n + 1) * 2,
        section_lateral_m=np.tile([-1.0, 2.0], n),
        section_normal=np.tile([0.0, 0, 1], (2 * n, 1)),
        section_normal_valid=np.ones(2 * n, bool),
        section_valid=np.ones(2 * n, bool),
        section_surface_index=np.tile([0, 1], n),
        center_surface_index=np.zeros(n, int),
        chosen_limits_m=np.tile([-1.0, 2.0], (n, 1)),
        physical_limits_m=np.tile([-1.0, 2.0], (n, 1)),
        physical_limits_valid=np.tile([True, False], (n, 1)),
    )
    np.savez_compressed(tmp_path / "route.npz", **a)
    m = dict(
        schema_version="0.3-track-samples",
        source={"mission": "/levels/test/main.level.json"},
        topology="closed" if closed else "open",
        query_construction="Test sections",
        limits_convention="Chosen",
        surfaces=[{"ground_model_name": "A"}, {"ground_model_name": "B"}],
        unavailable=[],
        route_sha256=digest(tmp_path / "route.npz"),
    )
    (tmp_path / "metadata.json").write_text(json.dumps(m))
    r = dict(
        schema_version="0.1-route-selection",
        frame="local_ENU",
        name="chosen",
        route_name=tmp_path.name,
        source_metadata_sha256=digest(tmp_path / "metadata.json"),
        source_route_sha256=digest(tmp_path / "route.npz"),
        start_row=1,
        end_row=None,
        direction="reverse",
        full_loop=closed,
    )
    return a, m, dict(topology=m["topology"]), r


def test_reverse_loop_rotates_seam_swaps_sides_and_retains_world_normals(tmp_path):
    a, _, route, r = source(tmp_path)
    b = reorder(a, route, r)
    np.testing.assert_array_equal(b["source_row"], [1, 0, 2, 1])
    np.testing.assert_array_equal(b["station_id"], [11, 10, 12, 11])
    np.testing.assert_array_equal(b["section_xyz"][:2], a["section_xyz"][[3, 2]])
    np.testing.assert_array_equal(b["section_surface_index"][:2], [1, 0])
    np.testing.assert_array_equal(b["section_lateral_m"][:2], [-2, 1])
    np.testing.assert_array_equal(b["chosen_limits_m"][0], [-2, 1])
    np.testing.assert_array_equal(b["physical_limits_valid"][0], [False, True])
    np.testing.assert_array_equal(b["section_normal"], np.tile([0, 0, 1], (8, 1)))
    np.testing.assert_array_equal(b["center_xyz"][0], b["center_xyz"][-1])


def test_open_sections_never_invent_a_loop_or_wrap(tmp_path):
    a, _, route, r = source(tmp_path, False)
    r.update(start_row=2, end_row=0, full_loop=False)
    np.testing.assert_array_equal(selection_rows(a, route, r), [2, 1, 0])
    r["direction"] = "forward"
    with pytest.raises(ValueError, match="reachable"):
        selection_rows(a, route, r)
    r["full_loop"] = True
    with pytest.raises(ValueError, match="explicitly closed"):
        selection_rows(a, route, r)


def test_materialization_is_standalone_and_source_hashes_guard_choices(tmp_path):
    _, _, _, r = source(tmp_path)
    original = (tmp_path / "route.npz").read_bytes()
    output = tmp_path / "exports/chosen"
    materialize(tmp_path, r, output)
    assert (tmp_path / "route.npz").read_bytes() == original
    assert (output / "inspect.html").exists()
    from route_json import export_route_json

    export_route_json(output, output / "road.json")
    d = json.loads((output / "road.json").read_text())
    assert d["route"]["start_station_id"] == 11
    r["source_route_sha256"] = "changed"
    with pytest.raises(ValueError, match="different source"):
        validate_recipe(tmp_path, r)


def test_context_retains_layers_and_does_not_include_remote_points():
    points = np.array([[1.0, 0, 0], [1, 0, 20], [10, 0, 0], [np.nan, 0, 0]])
    np.testing.assert_array_equal(
        nearby(points, np.array([[0.0, 0, 0]]), 2), [True, True, False, False]
    )


def test_reversal_preserves_known_fold_warnings_and_missing_samples(tmp_path):
    a, _, route, r = source(tmp_path)
    a["folded_xy_cell"] = np.array([[True], [False], [True]])
    a["section_xyz"][2, 2] = np.nan
    a["section_valid"][2] = False
    b = reorder(a, route, r)
    np.testing.assert_array_equal(b["folded_xy_cell"], [[True], [True], [False]])
    assert np.isnan(b["section_xyz"][1, 2]) and not b["section_valid"][1]
    a["center_xyz"][0, 2] = np.nan
    with pytest.raises(ValueError, match="seam gap"):
        selection_rows(a, route, r)


def test_selected_route_keeps_acquisition_recipe_separate_from_row_choice(tmp_path):
    _, meta, _, recipe = source(tmp_path)
    intent = tmp_path / "source_intent.json"
    intent.write_text('{"reference_operations":[]}\n')
    meta["reference_design"] = dict(
        recipe_file=intent.name, recipe_sha256=digest(intent), operations=[]
    )
    (tmp_path / "metadata.json").write_text(json.dumps(meta))
    recipe["source_metadata_sha256"] = digest(tmp_path / "metadata.json")
    output = tmp_path / "exports/selected"
    materialize(tmp_path, recipe, output)
    selected = json.loads((output / "metadata.json").read_text())
    acquired = output / selected["reference_design"]["recipe_file"]
    assert acquired.name == "source_reference_recipe.json" and digest(
        acquired
    ) == digest(intent)
    assert json.loads((output / "recipe.json").read_text()) == recipe


def recipe():
    return dict(
        reference_xyz=[[0, 0, 2], [1, 0, 2], [2, 0, 2]],
        reference_distance_xyz_m=[0, 1, 2],
        topology="open",
        settings=dict(
            left_extent_m=1,
            right_extent_m=2,
            extra_margin_m=0.5,
            station_spacing_m=1,
            lateral_spacing_m=0.5,
            ray_above_m=1,
            ray_below_m=2,
        ),
    )


def test_enu_left_and_right_order_and_margin():
    r = recipe()
    q, lat, left, fold = corridor_queries(r)
    np.testing.assert_array_equal(left, np.tile([0, 1, 0], (3, 1)))
    np.testing.assert_array_equal(q[:, :, 1], np.tile(lat, (3, 1)))
    assert lat[0] == -2.5 and lat[-1] == 1.5 and 0 in lat and not fold.any()
    reverse = copy.deepcopy(r)
    reverse["reference_xyz"].reverse()
    qr, lr, _, _ = corridor_queries(reverse)
    np.testing.assert_array_equal(qr[:, :, 1], np.tile(-lr, (3, 1)))


def test_closed_seam_has_same_section_orientation():
    r = recipe()
    r.update(
        topology="closed",
        reference_xyz=[[0, 0, 2], [10, 0, 2], [10, 10, 2], [0, 10, 2], [0, 0, 2]],
    )
    q, _, _, _ = corridor_queries(r)
    np.testing.assert_array_equal(q[0], q[-1])


def test_undefined_uturn_is_rejected_and_wide_corner_reports_fold():
    r = recipe()
    r["reference_xyz"] = [[0, 0, 2], [1, 0, 2], [0, 0, 2]]
    with pytest.raises(ValueError, match="tangent"):
        corridor_queries(r)
    r = recipe()
    r["reference_xyz"] = [[0, 0, 2], [1, 0, 2], [1, 1, 2]]
    assert corridor_queries(r)[3].any()


def test_missing_collision_does_not_become_reference_height(tmp_path):
    r = recipe()
    q, lat, left, fold = corridor_queries(r)
    source = dict(source={"mission": "/levels/test/main.level.json"}, files_sha256={})
    out = tmp_path / "track"
    # Rendering requires the recipe editor fields; arrays/validation are saved
    # before rendering. Provide a small complete intent snapshot.
    r.update(
        mode="free",
        waypoints=[{"xyz": [0, 0, 2]}, {"xyz": [2, 0, 2]}],
        editor={"settings": {}},
        source_mission=source["source"]["mission"],
    )
    report = save_track(
        out,
        r,
        source,
        q,
        lat,
        left,
        fold,
        [{"valid": False} for _ in q.reshape(-1, 3)],
        {},
    )
    assert report["valid_surface_hits"] == 0 and report["missing_center_stations"] == 3
    with np.load(out / "route.npz") as z:
        assert np.isnan(z["center_xyz"][:, 2]).all()
        assert not z["physical_limits_valid"].any()
        assert (z["section_surface_index"] == -1).all()
    from route_json import export_route_json
    import json

    export_route_json(out, out / "road.json")
    document = json.loads(
        (out / "road.json").read_text(),
        parse_constant=lambda _: pytest.fail("Nonstandard JSON NaN"),
    )
    assert document["arrays"]["center_xyz"][0][2] is None
    assert document["arrays"]["section_surface_index"][0] == -1


def fixture():
    path = dict(
        points=[[0, 0, 0], [4, 0, 0], [4, -0.15, 0], [4, 0.05, 0], [12, 0.05, 0]],
        segmentRoles=[
            "evaluated_guide",
            "native_graph_endpoint_connector",
            "native_graph_endpoint_connector",
            "evaluated_guide",
        ],
        segmentGuideIds=[7, 7, 7, 7],
        protectedPoints=[[0, 0, 0], [12, 0.05, 0]],
    )
    script = "const R=require(process.argv[1]);let s='';process.stdin.on('data',v=>s+=v);process.stdin.on('end',()=>process.stdout.write(JSON.stringify(R.shortNodeTransitions(JSON.parse(s)))));"
    out = subprocess.run(
        ["node", "-e", script, str(Path(__file__).with_name("route_planner.js"))],
        input=json.dumps(path),
        text=True,
        capture_output=True,
        check=True,
    )
    fixed = json.loads(out.stdout)
    recipe = dict(
        mode="network",
        source_reference_xyz=path["points"],
        source_reference_roles=path["segmentRoles"],
        navigation_edge_ids=[0] * 4,
        reference_operations=fixed["reference_operations"],
        waypoints=[dict(xyz=p) for p in path["protectedPoints"]],
    )
    return recipe, fixed, dict(edges=[dict(guide_id=7)])


def test_browser_reference_design_replays_without_moving_source_endpoints():
    recipe, fixed, geometry = fixture()
    replayed = replay_reference(recipe, geometry)
    np.testing.assert_allclose(replayed, fixed["points"], rtol=0, atol=1e-12)
    np.testing.assert_array_equal(
        replayed[[0, -1]], np.asarray(recipe["source_reference_xyz"])[[0, -1]]
    )
    assert np.min(replayed[:, 1]) >= -1e-8
    assert recipe["source_reference_xyz"][2][1] == -0.15


@pytest.mark.parametrize(
    "mutation", ["large_gap", "moved_waypoint", "wrong_role", "shifted_window"]
)
def test_replay_rejects_bypassing_reference_guards(mutation):
    recipe, _, geometry = fixture()
    recipe = copy.deepcopy(recipe)
    if mutation == "large_gap":
        recipe["reference_operations"][0]["settings"]["max_connector_length_m"] = 5
    elif mutation == "moved_waypoint":
        recipe["waypoints"].append(dict(xyz=recipe["source_reference_xyz"][1]))
    elif mutation == "wrong_role":
        recipe["source_reference_roles"][1] = "native_navigation_chord"
    else:
        recipe["reference_operations"][0]["transitions"][0]["start_m"] += 0.1
    with pytest.raises(ValueError):
        replay_reference(recipe, geometry)


def document(tmp_path):
    meta = {
        "schema_version": "0.2-native-survey",
        "source": {"mission": "/levels/test/main.level.json"},
        "files_sha256": {},
    }
    (tmp_path / "metadata.json").write_text(json.dumps(meta))
    return dict(
        schema_version="0.2-native-selection",
        frame="local_ENU",
        source_mission=meta["source"]["mission"],
        source_files_sha256={},
        bounds_enu_m=[0, 0, 0, 2, 2, 2],
        focus_xyz=[1, 1, 1],
    )


def test_named_save_is_inside_export_and_previous_version_is_kept(tmp_path):
    d = document(tmp_path)
    path = save_document(tmp_path, "areas", "parking", d)
    assert path == tmp_path / "selections/areas/parking.json"
    d["note"] = "revised"
    save_document(tmp_path, "areas", "parking", d)
    assert json.loads(path.read_text())["note"] == "revised"
    history = list((tmp_path / "selections/.history/areas").glob("*.json"))
    assert len(history) == 1 and "note" not in json.loads(history[0].read_text())


def test_other_export_and_path_traversal_are_rejected(tmp_path):
    d = document(tmp_path)
    d["source_files_sha256"] = {"samples": "different"}
    with pytest.raises(ValueError, match="different"):
        save_document(tmp_path, "areas", "wrong", d)
    for name in ["../outside", "/tmp/file", "a/b", "..", "bad name"]:
        with pytest.raises(ValueError):
            selection_path(tmp_path, "areas", name)
    with pytest.raises(ValueError):
        selection_path(tmp_path, "../other", "name")


def test_symlink_cannot_escape_export(tmp_path):
    outside = tmp_path / "outside"
    outside.mkdir()
    export = tmp_path / "export"
    export.mkdir()
    (export / "selections").symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="escapes"):
        selection_path(export, "areas", "name")


def snapshot(folder):
    """Capture bytes to detect mutation of published acquisitions."""
    return {
        p.relative_to(folder).as_posix(): p.read_bytes()
        for p in folder.rglob("*")
        if p.is_file()
    }


def test_migration_publishes_separate_bundle_and_rolls_back_failures(
    tmp_path, monkeypatch
):
    from route_schema import migrate_export
    import viewer

    original = tmp_path / "original"
    original.mkdir()
    source(original)
    before = snapshot(original)
    destination = tmp_path / "normalized"
    migrate_export(original, destination)
    assert snapshot(original) == before
    assert load_route(destination)[2]["name"] == original.name
    assert validate_routes(destination, write_report=False)["routes"][0][
        "array_checks_passed"
    ]
    with pytest.raises(FileExistsError):
        migrate_export(original, destination)
    with pytest.raises(ValueError, match="separate"):
        migrate_export(original, original / "nested")

    def fail_render(*args):
        raise RuntimeError("injected render failure")

    monkeypatch.setattr(viewer, "write_viewer", fail_render)
    failed = tmp_path / "failed"
    with pytest.raises(RuntimeError, match="injected"):
        migrate_export(original, failed)
    assert not failed.exists() and snapshot(original) == before


def test_failed_metadata_serialization_preserves_acquisition(tmp_path, monkeypatch):
    import route_schema

    source(tmp_path)
    before = snapshot(tmp_path)

    def fail_json(*args, **kwargs):
        raise ValueError("injected serialization failure")

    monkeypatch.setattr(route_schema.json, "dumps", fail_json)
    with pytest.raises(ValueError, match="injected"):
        normalize_export(tmp_path)
    assert snapshot(tmp_path) == before


def test_unknown_route_schema_cannot_be_read_or_exported(tmp_path):
    from route_json import export_route_json

    source(tmp_path)
    meta = json.loads((tmp_path / "metadata.json").read_text())
    meta["schema_version"] = "999-unsupported"
    (tmp_path / "metadata.json").write_text(json.dumps(meta))
    for operation in [
        lambda: load_route(tmp_path),
        lambda: export_route_json(tmp_path, tmp_path / "road.json"),
    ]:
        with pytest.raises(ValueError, match="Unsupported route schema"):
            operation()
    assert not (tmp_path / "road.json").exists()


@pytest.mark.parametrize(
    "mutation",
    ["nan_normal", "bad_index", "float_index", "wrong_shape", "degenerate", "nonunit"],
)
def test_bad_mesh_rejects_validation_and_json_publication(tmp_path, mutation):
    from route_json import export_route_json

    source(tmp_path)
    meta = normalize_export(tmp_path)
    mesh = dict(
        vertices_xyz=np.array([[0.0, 0, 0], [1, 0, 0], [0, 1, 0]]),
        triangles=np.array([[0, 1, 2]]),
        face_normal=np.array([[0.0, 0, 1]]),
    )
    if mutation == "nan_normal":
        mesh["face_normal"][:] = np.nan
    elif mutation == "bad_index":
        mesh["triangles"][0, 2] = 9
    elif mutation == "float_index":
        mesh["triangles"] = mesh["triangles"].astype(float)
    elif mutation == "wrong_shape":
        mesh["vertices_xyz"] = np.zeros((3, 2))
    elif mutation == "degenerate":
        mesh["vertices_xyz"][2] = mesh["vertices_xyz"][1]
    else:
        mesh["face_normal"] *= 2
    np.savez_compressed(tmp_path / "geometry.npz", **mesh)
    meta["geometry_sha256"] = digest(tmp_path / "geometry.npz")
    (tmp_path / "metadata.json").write_text(json.dumps(meta))
    for operation in [
        lambda: validate_routes(tmp_path),
        lambda: export_route_json(tmp_path, tmp_path / "road.json", include_mesh=True),
    ]:
        with pytest.raises(ValueError, match="[Mm]esh"):
            operation()
    assert not (tmp_path / "road.json").exists()


@pytest.mark.parametrize(
    "mutation",
    [
        "missing_reference",
        "missing_settings",
        "unknown_setting",
        "negative_spacing",
        "bad_topology",
        "changed_intent",
    ],
)
def test_editor_and_extractor_share_recipe_rejection(tmp_path, mutation):
    from track_recipe import load_recipe
    from local_server import validate_document

    meta = {
        "schema_version": "0.2-native-survey",
        "source": {"mission": "test"},
        "files_sha256": {},
    }
    (tmp_path / "metadata.json").write_text(json.dumps(meta))
    document = recipe() | dict(
        schema_version="0.3-track-recipe",
        frame="local_ENU",
        source_mission="test",
        source_files_sha256={},
        mode="free",
        source_reference_xyz=[[0, 0, 2], [2, 0, 2]],
    )
    validate_document(tmp_path, "tracks", document)
    if mutation == "missing_reference":
        del document["reference_xyz"]
    elif mutation == "missing_settings":
        del document["settings"]
    elif mutation == "unknown_setting":
        document["settings"]["typo"] = 1
    elif mutation == "negative_spacing":
        document["settings"]["lateral_spacing_m"] = -1
    elif mutation == "bad_topology":
        document["topology"] = "automatic"
    else:
        document["reference_xyz"][1][0] += 0.1
    downloaded = tmp_path / "recipe.json"
    downloaded.write_text(json.dumps(document))
    for operation in [
        lambda: save_document(tmp_path, "tracks", "bad", document),
        lambda: load_recipe(tmp_path, downloaded),
    ]:
        with pytest.raises(ValueError):
            operation()
    assert not (tmp_path / "selections").exists()


def test_portable_json_preserves_downstream_observations_without_mutating_source(
    tmp_path,
):
    from route_json import export_route_json

    source(tmp_path)
    normalize_export(tmp_path)
    arrays, _, definition, _ = load_route(tmp_path)
    before = snapshot(tmp_path)
    output = tmp_path.parent / (tmp_path.name + ".json")
    export_route_json(tmp_path, output)
    assert snapshot(tmp_path) == before
    document = json.loads(output.read_text())
    assert document["schema_version"] == "0.1-road-route-json"
    assert document["frame"] == "local_ENU" and document["units"] == "m"
    assert document["route"]["topology"] == definition["topology"]
    for key in [
        "center_xyz",
        "station_id",
        "section_xyz",
        "section_offsets",
        "section_normal",
        "section_normal_valid",
    ]:
        np.testing.assert_array_equal(document["arrays"][key], arrays[key])
        assert document["array_shapes"][key] == list(arrays[key].shape)


@pytest.mark.parametrize(
    "field,value", [("frame", "native"), ("units", "feet"), ("topology", "automatic")]
)
def test_publication_rejects_unsupported_frame_units_and_topology(
    tmp_path, field, value
):
    from route_json import export_route_json

    source(tmp_path)
    meta = normalize_export(tmp_path)
    if field == "topology":
        meta["routes"][0][field] = value
    else:
        meta[field] = value
    (tmp_path / "metadata.json").write_text(json.dumps(meta))
    with pytest.raises(ValueError):
        export_route_json(tmp_path, tmp_path / "road.json")
    assert not (tmp_path / "road.json").exists()
