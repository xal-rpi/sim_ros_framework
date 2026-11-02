"""Replay bounded reference-design operations; never modify surface observations."""

import numpy as np


def replay_reference(recipe, guide_geometry=None):
    source = np.asarray(recipe["source_reference_xyz"], float)
    operations = recipe.get("reference_operations", [])
    if not operations:
        return source
    if (
        len(operations) != 1
        or operations[0].get("op") != "short_same_guide_node_transition"
        or operations[0].get("version") != 1
    ):
        raise ValueError("Unsupported reference design operation")
    if recipe["mode"] != "network" or guide_geometry is None:
        raise ValueError("Node transitions require verified native guide associations")
    operation = operations[0]
    settings = operation["settings"]
    if set(settings) != {
        "window_m",
        "max_connector_length_m",
        "max_heading_deg",
        "sample_spacing_m",
    }:
        raise ValueError("Invalid reference transition settings")
    if (
        not all(np.isfinite(v) and v > 0 for v in settings.values())
        or settings["max_connector_length_m"] > 0.5
        or settings["max_heading_deg"] > 30
    ):
        raise ValueError("Reference transition exceeds bounded policy")
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(source, axis=0), axis=1))]
    roles = recipe["source_reference_roles"]
    ids = recipe["navigation_edge_ids"]
    if len(roles) != len(source) - 1:
        raise ValueError("Missing reference segment roles")

    def at(s):
        i = min(np.searchsorted(arc, s, side="right") - 1, len(source) - 2)
        delta = source[i + 1] - source[i]
        length = np.linalg.norm(delta)
        if length <= 1e-10:
            raise ValueError("Undefined transition tangent")
        return source[i] + delta * ((s - arc[i]) / length), delta / length

    result = []
    cursor = 0
    previous_end = -np.inf

    def append(p):
        if not result or np.linalg.norm(result[-1] - p) > 1e-10:
            result.append(p)

    for edit in operation["transitions"]:
        segments = edit["connector_segments"]
        start, end = edit["start_m"], edit["end_m"]
        if not segments or any(type(i) is not int for i in segments):
            raise ValueError("Invalid connector indices")
        first, last = segments[0], segments[-1]
        if (
            segments != list(range(first, last + 1))
            or first <= 0
            or last >= len(roles) - 1
        ):
            raise ValueError("Transition must have source geometry on both sides")
        if (
            any(roles[i] != "native_graph_endpoint_connector" for i in segments)
            or roles[first - 1] != "evaluated_guide"
            or roles[last + 1] != "evaluated_guide"
        ):
            raise ValueError("Transition is not a bounded connector group")
        before, after = (
            guide_geometry["edges"][ids[first - 1]],
            guide_geometry["edges"][ids[last + 1]],
        )
        if before["guide_id"] is None or before["guide_id"] != after["guide_id"]:
            raise ValueError("Transition cannot connect separate or unknown guides")
        if arc[last + 1] - arc[first] > settings["max_connector_length_m"] + 1e-9:
            raise ValueError("Connector group exceeds reference policy")
        if (
            abs(start - (arc[first] - settings["window_m"])) > 1e-8
            or abs(end - (arc[last + 1] + settings["window_m"])) > 1e-8
            or start < 0
            or end > arc[-1]
            or start < previous_end
        ):
            raise ValueError("Invalid or overlapping reference transition interval")
        for i, role in enumerate(roles):
            if arc[i] < end and arc[i + 1] > start and i not in segments:
                if (
                    role != "evaluated_guide"
                    or guide_geometry["edges"][ids[i]]["guide_id"] != before["guide_id"]
                ):
                    raise ValueError(
                        "Transition window contains another junction or source"
                    )
        for waypoint in recipe["waypoints"]:
            if np.any(
                (np.linalg.norm(source - np.asarray(waypoint["xyz"]), axis=1) < 1e-8)
                & (arc > start + 1e-8)
                & (arc < end - 1e-8)
            ):
                raise ValueError("Reference transition would move a selected waypoint")
        a, ta = at(start)
        b, tb = at(end)
        den = np.linalg.norm(ta[:2]) * np.linalg.norm(tb[:2])
        angle = np.degrees(np.arccos(np.clip(ta[:2] @ tb[:2] / max(den, 1e-20), -1, 1)))
        if (
            den < 1e-10
            or angle > settings["max_heading_deg"] + 1e-8
            or abs(ta[2] - tb[2]) > 0.1
        ):
            raise ValueError("Reference heading or grade requires explicit design")
        count = edit["sample_count"]
        if (
            type(count) is not int
            or count != int(np.ceil((end - start) / settings["sample_spacing_m"]))
            or count > 10000
        ):
            raise ValueError("Invalid transition sample count")
        while cursor < len(source) and arc[cursor] < start - 1e-10:
            append(source[cursor])
            cursor += 1
        scale = np.linalg.norm(b - a)
        v0, v1 = ta * scale, tb * scale
        delta = b - a
        for t in np.linspace(0, 1, count + 1):
            append(
                a
                + v0 * t
                + (10 * delta - 6 * v0 - 4 * v1) * t**3
                + (-15 * delta + 8 * v0 + 7 * v1) * t**4
                + (6 * delta - 3 * v0 - 3 * v1) * t**5
            )
        while cursor < len(source) and arc[cursor] <= end + 1e-10:
            cursor += 1
        previous_end = end
    while cursor < len(source):
        append(source[cursor])
        cursor += 1
    return np.asarray(result)
