# Road geometry live validation — 2026-11-01

The current editor → live extraction → measured-route selection → trajgen fitting
workflow passed against a running BeamNG instance. No production code change was
needed during this check. This exercises one short open Derby corridor, not
arbitrary map geometry or vehicle execution.

Artifacts were retained locally under
`environments/dist/road_geometry/validation_20261101/`. This ignored output directory
is not included in the source checkpoint.
`validation_manifest.json` records the source-file hashes, Git HEAD and report
locations; the tested working tree contains uncommitted changes.

## Scene and acquisition

BeamNG was initially at the menu. The check loaded a paused, vehicle-free Derby
scene (`/levels/derby/main.level.json`). No vehicle was spawned or driven.

```bash
python3 tools/road_geometry/cli.py survey --fresh --level derby \
  --bounds 105 -45 70 140 -15 90 --spacing 2 --output NEW_ROOT/survey
python3 tools/road_geometry/cli.py prepare NEW_ROOT/survey \
  --spacing 20 --height-band -50 200
```

The bounded survey returned 288/288 static heights and 253 matching normals.
Its independent midpoint diagnostics exposed large errors across changing surface
objects (5.27 m maximum), consistent with the overview remaining visual context;
within cells with matching corner-object identities the maximum was 0.152 m.
Neither result is a bound on between-sample geometry.

## Browser and reference workflow

Headless Chromium driven by Playwright exercised the actual rendered pages and
loopback HTTP service, including map pointer clicks and visible form controls.
Browser scripts ran from `/tmp/road-browser/`; no new repository test file or
browser dependency was added.

- Two clicks selected an evaluated-guide network path on directed edge `[60,61]`,
  guide 54, approximately 10 m long with 12 stations.
- Saving and reopening `live_network` preserved the reference preview exactly.
- The map/3D controls worked without browser JavaScript errors.
- A malformed recipe POST returned HTTP 400 and was not saved.
- Offline `rebuild-reference` retained waypoint positions exactly, producing 12
  stations and no unresolved transitions. This path needed zero reference
  transitions; the live check did not exercise a nonzero transition.
- The measured-route viewer saved/reopened a reverse choice, created
  `corridor/exports/live_reverse`, and opened its generated inspector.
- The original measured `route.npz` hash remained unchanged after export.

`browser_check.json`, `route_browser_check.json`, `editor.png` and
`measured_route.png` retain the browser evidence.

## Live corridor and downstream fitting

The browser recipe requested left/right extents of 1 m, 0.5 m extra margin,
1 m station spacing, 0.5 m lateral spacing and a vertical ray band of ±2 m.

```bash
python3 tools/road_geometry/cli.py extract-path NEW_ROOT/survey \
  NEW_ROOT/rebuilt_network.json --output NEW_ROOT/corridor
```

| Check | Result |
|---|---|
| Measured stations / lateral samples | 12 / 84 |
| Valid static heights | 84/84; no missing centers |
| Matching native normals | 73/84; others remained invalid |
| Folded sampling cells / lateral height jumps over 0.3 m | 0 / 0 |
| Canonical observation validation | Passed |
| Reverse-route JSON → actual trajgen adapter → save/load observations | Passed; coordinates and normals/masks preserved |
| `prepare_track` representation / grid target | `plan_view_height` / 0.1 m |
| Maximum sampled XYZ error / preset rejection limit | 0.016546 m / 0.05 m |
| Maximum normal disagreement | 8.065°; diagnostic, not the XYZ acceptance gate |
| Chosen fitting domain / sampled coverage | ±1 m / approximately ±1.5 m throughout |

The fitter was called with explicit `seam_policy='reject'`; the open route had no
duplicate stations. `trajgen_check.json` records the detailed fit and coverage
results. Observation arrays were not smoothed or resampled by the exporter;
trajgen constructed its fitted chart separately.

No trajectory optimization, controller compilation, vehicle execution, closed-loop
seam handling or collision certification of the fitted surface was performed.
The Python/JavaScript regression suite remains separate from this live check.
