"""Small deduplicated surface table; unknown names are JSON null, never inferred."""

import json

import numpy as np


def surface_record(properties, object_fields=None):
    properties, obj = properties or {}, object_fields or {}
    return dict(
        kind=properties.get("kind", "unknown"),
        object_name=obj.get("engineObjectName", obj.get("engineObject")) or None,
        object_class=obj.get("engineObjectClass"),
        object_runtime_id=obj.get("engineObjectId"),
        object_persistent_id=obj.get("engineObjectPersistentId") or None,
        material_name=properties.get("material_name") or None,
        terrain_material_index=properties.get("terrain_material_index"),
        ground_model_name=properties.get("ground_model_name") or None,
        resolution_source=properties.get("resolution_source", "unknown"),
    )


def intern_surface(table, indices, record):
    key = json.dumps(record, sort_keys=True)
    if key not in indices:
        indices[key] = len(table)
        table.append(record)
    return indices[key]


def pack_surface_properties(hits, matching):
    """Reject Engine ray properties when its surface differs from static height."""
    table, lookup = [], {}
    indices = np.full(len(hits), -1, np.int64)
    for i, hit in enumerate(hits):
        if matching[i] and hit.get("surfaceProperties"):
            indices[i] = intern_surface(
                table, lookup, surface_record(hit["surfaceProperties"], hit)
            )
    return indices, table


def validate_surface_indices(indices, table, valid=None):
    indices = np.asarray(indices)
    return bool(
        np.issubdtype(indices.dtype, np.integer)
        and ((indices >= -1) & (indices < len(table))).all()
        and (valid is None or ((indices == -1) | np.asarray(valid, bool)).all())
    )
