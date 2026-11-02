"""Geometry acquisition, observations and guide association checks."""

import copy
import guide_geometry as G
import json
import numpy as np
import pytest
from geometry import (
    HeightIndex,
    load_export,
    pack,
    route_endpoints,
    select_recipe,
    validate,
    write_export,
)
from native_scene import (
    grid_queries,
    interpolation_diagnostics,
    pack_guides,
    pack_observations,
)
from surface_properties import pack_surface_properties, validate_surface_indices


@pytest.fixture
def segment():
    points = [dict(index=i + 1, xyz=[2 * i, 0, 1], up=[0, 0, 1]) for i in range(2)]
    return dict(
        piece=2,
        subtrack=1,
        quality=1,
        centerMesh="flat",
        objectId=10,
        objectName="test",
        points=points,
        meshes=[
            dict(
                index=1,
                tag="center",
                material="test",
                vertices=[[0, -1, 0.8], [0, 1, 0.8], [2, -1, 0.8], [2, 1, 0.8]],
                triangles=[[0, 1, 2], [2, 1, 3]],
                normals=[[0, 0, 1]],
                normalCorners=[[0, 0, 0], [0, 0, 0]],
            )
        ],
    )


def test_winding_preserved_and_physical_center_not_design_curve(segment):
    g, _ = pack([segment])
    np.testing.assert_array_equal(g["triangles"], segment["meshes"][0]["triangles"])
    np.testing.assert_allclose(g["face_normal"], [[0, 0, 1], [0, 0, 1]])


def test_vertical_queries_retain_height_layers(segment):
    higher = copy.deepcopy(segment)
    higher["piece"] = 3
    for v in higher["meshes"][0]["vertices"]:
        v[2] += 5
    g, _ = pack([segment, higher])
    index = HeightIndex(g)
    ids, z = index.hits(0.5, 0)
    assert set(np.round(z, 4)) == {0.8, 5.8}
    assert index.height([0.5, 0, 0.8])[0] == pytest.approx(0.8)
    assert index.height([0.5, 0, 5.8])[0] == pytest.approx(5.8)
    assert index.height([0.5, 0, 3]) is None


def test_export_routes_and_sections_use_evaluated_vertices(tmp_path, segment):
    out = tmp_path / "export"
    write_export(
        out,
        dict(asset_name="test"),
        [segment],
        lua_hash="lua",
        asset_hash="asset",
        asset_bytes=b"{}",
    )
    g, _ = load_export(out)
    with np.load(out / "routes/subtrack_1.npz", allow_pickle=False) as route:
        np.testing.assert_allclose(route["center_xyz"][:, 2], 0.8)
        np.testing.assert_allclose(route["reference_xyz"][:, 2], 1)
        assert not route["physical_limits_valid"].any()
        np.testing.assert_allclose(
            route["section_xyz"], g["vertices_xyz"][route["section_vertex_id"]]
        )
    report = validate(out)
    assert report["routes"][0]["checks"]["sections_ordered"]
    assert report["routes"][0]["endpoint_gap_m"] == 2
    assert report["routes"][0]["topology"] == "unknown"


def test_selection_never_joins_disconnected_station_ranges(tmp_path, segment):
    out = tmp_path / "export"
    _, meta = write_export(
        out,
        dict(asset_name="test"),
        [segment],
        lua_hash="lua",
        asset_hash="asset",
        asset_bytes=b"{}",
    )
    route_path = out / "routes/subtrack_1.npz"
    np.savez_compressed(
        route_path,
        station_id=[10, 20, 30],
        center_xyz=[[0, 0, 0], [10, 10, 0], [1, 0, 0]],
        source_ref=np.zeros((3, 3), int),
        distance_xyz_m=[0, 14, 28],
        section_offsets=[0, 2, 4, 6],
        section_xyz=np.zeros((6, 3)),
        section_lateral_m=[-1, 1, -1, 1, -1, 1],
    )
    recipe = dict(
        geometry_sha256=meta["geometry_sha256"],
        frame="local_ENU",
        operations=[
            dict(op="select_route", name="subtrack_1"),
            dict(op="slice_stations", first=10, last=30),
            dict(op="spatial_box", min_xyz=[-1, -1, -1], max_xyz=[2, 1, 1]),
        ],
    )
    selected = tmp_path / "selected"
    report = select_recipe(out, recipe, selected)
    assert report["parts"] == 2
    with np.load(selected / "selection.npz", allow_pickle=False) as z:
        np.testing.assert_array_equal(z["station_id"], [10, 30])
        np.testing.assert_array_equal(z["part_id"], [0, 1])
    recipe["geometry_sha256"] = "wrong export"
    with pytest.raises(ValueError, match="different immutable"):
        select_recipe(out, recipe, tmp_path / "rejected")


def test_export_hash_detects_tampering(tmp_path, segment):
    out = tmp_path / "export"
    write_export(
        out,
        dict(asset_name="test"),
        [segment],
        lua_hash="lua",
        asset_hash="asset",
        asset_bytes=b"{}",
    )
    with (out / "geometry.npz").open("ab") as f:
        f.write(b"altered")
    with pytest.raises(ValueError):
        load_export(out)


def test_managed_material_properties_survive_route_and_selection(tmp_path, segment):
    segment["meshes"][0]["surfaceProperties"] = dict(
        kind="managed_trackbuilder",
        material_name="test",
        ground_model_name="ASPHALT_OLD",
        resolution_source="evaluated_mesh_material",
    )
    out = tmp_path / "export"
    geometry, meta = write_export(
        out, {}, [segment], lua_hash="lua", asset_hash="asset", asset_bytes=b"{}"
    )
    assert meta["surfaces"][0]["ground_model_name"] == "ASPHALT_OLD"
    np.testing.assert_array_equal(geometry["triangle_surface_index"], [0, 0])
    recipe = dict(
        geometry_sha256=meta["geometry_sha256"],
        frame="local_ENU",
        operations=[dict(op="select_route", name="subtrack_1")],
    )
    select_recipe(out, recipe, tmp_path / "selected")
    with np.load(tmp_path / "selected/selection.npz") as z:
        np.testing.assert_array_equal(z["center_surface_index"], [0, 0])
        np.testing.assert_array_equal(z["section_surface_index"], [0, 0, 0, 0])
    from route_json import export_route_json

    export_route_json(out, out / "road.json", include_mesh=True)
    document = json.loads((out / "road.json").read_text())
    assert document["surfaces"][0]["ground_model_name"] == "ASPHALT_OLD"
    assert document["route"]["topology"] == "unknown"
    assert document["arrays"]["center_xyz"][0][2] == 0.8
    assert document["array_shapes"]["section_xyz"] == [4, 3]
    assert document["geometry_arrays"]["triangle_surface_index"] == [0, 0]
    with pytest.raises(FileExistsError):
        export_route_json(out, out / "road.json")


def test_closed_route_requires_explicit_choice_and_valid_native_seam():
    center = np.array([[0.0, 0, 1], [1.0, 0, 1], [0.0, 0, 1]])
    forward = np.tile([1.0, 0, 0], (3, 1))
    up = np.tile([0.0, 0, 1], (3, 1))
    info = dict(declaredConnected=True, evaluatedClosed=True)
    defaults = dict(
        topology="closed",
        start="first_evaluated_station",
        direction="native_builder_order",
    )
    assert route_endpoints(center, forward, up, {}, info)["topology"] == "unknown"
    route = route_endpoints(center, forward, up, defaults, info)
    assert (
        route["topology"] == "closed"
        and route["seam_station_id"] == 0
        and route["end_station_id"] == 2
    )
    bad = center.copy()
    bad[-1, 0] = 4
    with pytest.raises(ValueError, match="seam mismatch"):
        route_endpoints(bad, forward, up, defaults, info)
    bad = forward.copy()
    bad[-1] = [-1, 0, 0]
    with pytest.raises(ValueError, match="seam mismatch"):
        route_endpoints(center, bad, up, defaults, info)
    with pytest.raises(ValueError, match="builder closure"):
        route_endpoints(
            center,
            forward,
            up,
            defaults,
            dict(declaredConnected=False, evaluatedClosed=True),
        )


def test_grid_keeps_requested_spacing_and_explicit_height_band():
    xy, shape = grid_queries([0, 0, 70, 2.4, 1.4, 95], 1)
    assert shape == (2, 3)
    np.testing.assert_array_equal(xy, [[0, 0], [1, 0], [2, 0], [0, 1], [1, 1], [2, 1]])
    with pytest.raises(ValueError):
        grid_queries([0, 0, 5, 2, 2, 3], 1)
    with pytest.raises(ValueError, match="250,000"):
        grid_queries([0, 0, 0, 1000, 1000, 1], 1e-6)


def test_normal_and_object_are_unknown_when_ray_apis_disagree():
    xyz = [[0, 0, 3], [1, 0, 3], [2, 0, 3]]
    hits = [
        dict(
            valid=True,
            height=3,
            enginePoint=p,
            engineNormal=[0, 0, 1],
            engineObject="surface",
            engineObjectPersistentId="stable",
        )
        for p in xyz
    ]
    hits[1]["enginePoint"][2] = 4
    hits[2] = dict(valid=False, enginePoint=[2, 0, 3], engineNormal=[0, 0, 1])
    a, objects = pack_observations([[0, 0], [1, 0], [2, 0]], hits)
    np.testing.assert_array_equal(a["normal_valid"], [True, False, False])
    np.testing.assert_array_equal(a["surface_object_index"], [0, -1, -1])
    assert np.isnan(a["surface_normal"][1:]).all()
    assert objects[0]["engineObjectPersistentId"] == "stable"


def test_guides_keep_independent_paths_and_native_ownership():
    def road(pid, x):
        return dict(
            persistentId=pid,
            material="road_invisible",
            drivability="0.9",
            edges=[dict(right=[x, -1, 0], middle=[x, 0, 0], left=[x, 1, 0])],
        )

    a, sources = pack_guides({"20": road("b", 10), "10": road("a", 0)})
    np.testing.assert_array_equal(a["road_offsets"], [0, 1, 2])
    np.testing.assert_array_equal(a["edge_xyz"][0], [[0, -1, 0], [0, 0, 0], [0, 1, 0]])
    assert [s["persistent_id"] for s in sources] == ["a", "b"]


def test_independent_midpoint_exposes_a_height_discontinuity():
    xy, shape = grid_queries([0, 0, -1, 1, 1, 10], 1)
    hits = [dict(valid=True, height=z) for z in [0, 0, 0, 8]]
    a, _ = pack_observations(xy, hits)
    checks, _ = pack_observations([[0.5, 0.5]], [dict(valid=True, height=0)])
    report, error = interpolation_diagnostics(a, shape, np.array([[0.5, 0.5]]), checks)
    assert report["valid_checks"] == 1
    assert error[0] == 2
    assert report["same_corner_object_cells"] is None


def test_properties_follow_actual_hit_and_keep_terrain_ownership():
    terrain = dict(
        kind="terrain",
        material_name="Grass",
        terrain_material_index=0,
        ground_model_name="GRASS",
        resolution_source="ray_hit_terrain_material",
    )
    hits = [
        dict(surfaceProperties=terrain, engineObjectId=1),
        dict(surfaceProperties=terrain, engineObjectId=1),
        dict(surfaceProperties=terrain, engineObjectId=2),
        dict(surfaceProperties=terrain, engineObjectId=1),
        dict(
            surfaceProperties=dict(
                kind="mesh", resolution_source="ray_hit_object_only"
            ),
            engineObjectId=3,
        ),
    ]
    indices, table = pack_surface_properties(hits, [True, True, True, False, True])
    np.testing.assert_array_equal(indices, [0, 0, 1, -1, 2])
    assert len(table) == 3 and table[0]["ground_model_name"] == "GRASS"
    assert table[2]["material_name"] is None and table[2]["ground_model_name"] is None
    assert validate_surface_indices(indices, table, [True, True, True, False, True])
    assert not validate_surface_indices([3], table)
    assert not validate_surface_indices([0], table, [False])


def test_local_branch_is_used_instead_of_rejecting_a_long_global_projection(
    tmp_path, monkeypatch
):
    line = np.array(
        [[0.2, 0, 0], [0.2, 20, 0], [20, 20, 0], [20, -20, 0], [0, -20, 0], [0, 0, 0.0]]
    )
    # The nearest start projection is the other END of this open guide.
    a, b = np.array([0.05, 0, 0]), np.array([0.2, 20, 0])
    assert G.project(line, a)[0] == len(line) - 1
    np.savez(
        tmp_path / "road_guides.npz",
        edge_xyz=np.repeat(line[:, None, :], 3, axis=1),
        road_offsets=[0, len(line)],
    )
    (tmp_path / "navigation.json").write_text(
        json.dumps(
            dict(
                nodes=[dict(xyz=a.tolist()), dict(xyz=b.tolist())],
                edges=[{"from": 0, "to": 1}],
            )
        )
    )
    monkeypatch.setattr(
        G,
        "load_survey",
        lambda _: dict(
            files_sha256={}, roads=[dict(drivability=1, persistent_id="open-guide")]
        ),
    )
    g = G.build_guide_geometry(tmp_path)["edges"][0]
    assert g["guide_id"] == 0
    np.testing.assert_allclose(g["native_station_interval"], [0, 1])
    assert g["roles"] == ["native_graph_endpoint_connector", "evaluated_guide"]
    # The old snapshot is kept for saved recipes; new caches name their content.
    old = tmp_path / "navigation_geometry.json"
    old.write_text("old snapshot")
    path = G.write_guide_geometry(tmp_path)
    assert (
        old.read_text() == "old snapshot" and path.parent.name == "guide_associations"
    )
    assert path == G.write_guide_geometry(tmp_path)
