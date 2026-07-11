"""bng_simulator.utils.logger_utils

Helpers to load experiment logs produced by the sim framework.

The run folder layout is expected to look like:

- run_XXX/
    - metadata.yaml
    - data/data.pkl
    - rosbag_YYYYMMDD_HHMMSS/
        - metadata.yaml
        - *.mcap

This module provides:
- legacy helpers: :func:`load_metadata`, :func:`load_consolidated_data`, :func:`load_log_data`
- a higher-level loader: :func:`load_run_data` that merges pickle + rosbag (MCAP) data.
"""

from __future__ import annotations

import os
import pickle
import re
from pathlib import Path
from collections.abc import Mapping, Sequence
from numbers import Number
from typing import Any, Dict, Optional, Tuple, Union, List, Iterable

from bng_simulator.utils.io_dict_utils import load_yaml


def load_metadata(run_number, root_dir="~/beamng_log_data"):
    """
    Load metadata from the specified run number.

    Args:
        run_number (int): The run number (e.g., 1 for run_001).
        root_dir (str): The root directory where run folders are stored (default: ~/beamng_log_data).

    Returns:
        dict: The metadata dictionary loaded from metadata.yaml.

    Raises:
        FileNotFoundError: If the metadata file does not exist.
    """
    run_folder = os.path.join(os.path.expanduser(root_dir), f"run_{run_number:03d}")
    metadata_path = os.path.join(run_folder, "metadata.yaml")
    if not os.path.exists(metadata_path):
        raise FileNotFoundError(f"Metadata file not found: {metadata_path}")
    metadata = load_yaml(metadata_path)
    return metadata


def load_consolidated_data(run_number, root_dir="~/beamng_log_data"):
    """
    Load consolidated log data from the specified run number.

    Args:
        run_number (int): The run number (e.g., 1 for run_001).
        root_dir (str): The root directory where run folders are stored (default: ~/beamng_log_data).

    Returns:
        dict: The consolidated log data loaded from data.pkl.

    Raises:
        FileNotFoundError: If the consolidated data file does not exist.
    """
    run_folder = os.path.join(os.path.expanduser(root_dir), f"run_{run_number:03d}")
    data_file = os.path.join(run_folder, "data", "data.pkl")
    if not os.path.exists(data_file):
        raise FileNotFoundError(f"Consolidated data file not found: {data_file}")
    with open(data_file, "rb") as f:
        data = pickle.load(f)
    return data


def _resolve_run_path(
    *,
    run_number: Optional[int] = None,
    run_path: Optional[Union[str, os.PathLike]] = None,
    root_dir: Union[str, os.PathLike] = "~/beamng_log_data",
) -> Path:
    if (run_number is None) == (run_path is None):
        raise ValueError("Provide exactly one of run_number or run_path")

    if run_path is not None:
        path = Path(os.path.expanduser(str(run_path))).resolve()
    else:
        path = Path(os.path.expanduser(str(root_dir))).resolve() / f"run_{run_number:03d}"

    if not path.exists():
        raise FileNotFoundError(f"Run folder not found: {path}")
    if not path.is_dir():
        raise NotADirectoryError(f"Run path is not a directory: {path}")
    return path


def _load_consolidated_data_from_run_path(run_path: Path) -> Any:
    data_file = run_path / "data" / "data.pkl"
    if not data_file.exists():
        raise FileNotFoundError(f"Consolidated data file not found: {data_file}")
    with data_file.open("rb") as f:
        return pickle.load(f)


def _find_rosbag_dirs(run_path: Path) -> list[Path]:
    # The rosbag2 record command creates a directory. In your setup you used
    # rosbag_YYYYMMDD_HHMMSS, but we keep it a bit flexible.
    candidates: list[Path] = []
    for child in run_path.iterdir():
        if not child.is_dir():
            continue
        if not child.name.startswith("rosbag_"):
            continue
        if (child / "metadata.yaml").exists() or any(child.glob("*.mcap")):
            candidates.append(child)
    candidates.sort()
    return candidates


# Treat numpy scalars as scalar via Number; keep None too.
ScalarTypes = (str, int, float, bool, type(None))

def _is_sequence(x: Any) -> bool:
    # Avoid expanding strings/bytes as sequences
    return isinstance(x, Sequence) and not isinstance(x, (str, bytes, bytearray))


def _message_to_builtin(obj: Any) -> Any:
    """Best-effort conversion of decoded ROS messages to built-in Python types.

    This enables downstream flattening to work even when deserialization returns
    dataclasses or message-like objects.
    """

    if obj is None or isinstance(obj, (str, bytes, bytearray, int, float, bool)):
        return obj

    if isinstance(obj, Mapping):
        return {str(k): _message_to_builtin(v) for k, v in obj.items()}

    if _is_sequence(obj):
        return [_message_to_builtin(v) for v in obj]

    # dataclasses
    try:
        import dataclasses

        if dataclasses.is_dataclass(obj):
            return {k: _message_to_builtin(v) for k, v in dataclasses.asdict(obj).items()}
    except Exception:
        pass

    # slots-based objects
    slots = getattr(obj, "__slots__", None)
    if slots:
        out: Dict[str, Any] = {}
        for k in slots:
            if isinstance(k, str) and not k.startswith("_") and hasattr(obj, k):
                out[k] = _message_to_builtin(getattr(obj, k))
        if out:
            return out

    # plain objects
    d = getattr(obj, "__dict__", None)
    if isinstance(d, dict) and d:
        return {k: _message_to_builtin(v) for k, v in d.items() if not str(k).startswith("_")}

    return obj

def flatten_record(
    obj: Any,
    prefix: str = "",
    sep: str = "_",
    out: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Flatten nested dict-like + list/tuple-like structures into a flat dict.

    Examples:
      {"vel": {"x": 1, "y": 2}} -> {"vel_x": 1, "vel_y": 2}
      {"ranges": [10, 11]}      -> {"ranges_0": 10, "ranges_1": 11}
      {"a":[{"b":1},{"b":2}]}   -> {"a_0_b": 1, "a_1_b": 2}
    """
    if out is None:
        out = {}

    if isinstance(obj, Mapping):
        for k, v in obj.items():
            k = str(k)
            new_prefix = f"{prefix}{sep}{k}" if prefix else k
            flatten_record(v, new_prefix, sep, out)
        return out

    if _is_sequence(obj):
        for i, v in enumerate(obj):
            new_prefix = f"{prefix}{sep}{i}" if prefix else str(i)
            flatten_record(v, new_prefix, sep, out)
        return out

    # Leaf: scalar or other object (we keep as-is)
    # If the root object is a scalar (e.g. std_msgs/* with a single primitive
    # field, or undecoded raw bytes), `prefix` can be empty. Use a stable
    # default key instead of producing an empty-string key.
    key = prefix if prefix else "value"
    out[key] = obj
    return out

def _infer_fixed_keys(sample_msg: Mapping[str, Any], sep: str = "_") -> List[str]:
    """
    Infer the fixed schema key list from a representative sample message.
    Raises if the schema contains ambiguous empty prefix.
    """
    flat = flatten_record(sample_msg, sep=sep)
    if "" in flat:
        raise ValueError("Got an empty key during flattening. Check your input/prefix logic.")
    return list(flat.keys())

def _flatten_rosbag_messages(
    messages: Iterable[Mapping[str, Any]],
    *,
    sep: str = "_",
    fill_value: Any = None,
    keys: Optional[List[str]] = None,
    strict: bool = True,
) -> Dict[str, List[Any]]:
    """
    Flatten an iterable of fixed-schema messages into columns: key -> list(values).

    Args:
      messages: iterable of dict/OrderedDict-like messages
      sep: key separator (default "_")
      fill_value: used if a key is missing in a message (should be rare for fixed schema)
      keys: optionally provide the flattened key list (avoids inferring from first message)
      strict: if True, raises if any message introduces NEW keys not in 'keys'

        Returns:
            cols: dict mapping flattened key -> list of values (aligned by message index)
    """
    it = iter(messages)

    # Get first message to infer schema if needed
    first_msg = None
    if keys is None:
        try:
            first_msg = next(it)
        except StopIteration:
            return {}
        keys = _infer_fixed_keys(first_msg, sep=sep)

    # Initialize columns
    cols: Dict[str, List[Any]] = {k: [] for k in keys}

    def _append_from_flat(flat: Dict[str, Any]) -> None:
        # Optionally check for unexpected keys
        if strict:
            extra = set(flat.keys()) - set(cols.keys())
            if extra:
                raise KeyError(f"New keys encountered despite fixed schema: {sorted(extra)[:20]}")
        else:
            # If not strict, add new columns on the fly (and backfill)
            extra = set(flat.keys()) - set(cols.keys())
            if extra:
                n_rows = len(next(iter(cols.values()))) if cols else 0
                for k in extra:
                    cols[k] = [fill_value] * n_rows
                    keys.append(k)

        # Append values in fixed order
        for k in keys:
            cols[k].append(flat.get(k, fill_value))

    # Process first message if we consumed it
    if first_msg is not None:
        _append_from_flat(flatten_record(first_msg, sep=sep))

    # Process remaining messages
    for m in it:
        _append_from_flat(flatten_record(m, sep=sep))

    return cols

def _try_load_rosbag_messages(
    bag_dir: Path,
    *,
    max_messages_per_topic: Optional[int] = None,
    decode_messages: bool = True,
    topic_filter: Optional[callable] = None,
) -> Dict[str, Dict[str, Any]]:
    """Best-effort rosbag2 reader.

    Returns a dict: topic -> {"type": str, "timestamps_ns": [...], "messages": [...]}
    
    Args:
        bag_dir: Path to the rosbag directory
        max_messages_per_topic: Optional limit on messages per topic
        decode_messages: Whether to deserialize messages or keep raw bytes
        topic_filter: Optional callable to filter topics (default: None = load all topics)
    """

    # Prefer schema-based decoding (no ROS2 msg packages required) via `rosbags`, if available.
    try:
        from rosbags.rosbag2 import Reader as RosbagsReader  # type: ignore
        from rosbags.typesys import Stores, get_typestore  # type: ignore

        out: Dict[str, Dict[str, Any]] = {}
        counts: Dict[str, int] = {}

        with RosbagsReader(str(bag_dir)) as reader:
            # Many rosbags versions provide a typestore populated from the bag.
            typestore = getattr(reader, "typestore", None)
            if typestore is None:
                typestore = get_typestore(Stores.ROS2_HUMBLE)

            # rosbags API compatibility:
            # - Older rosbags exported `deserialize_cdr` from `rosbags.serde`.
            # - rosbags==0.11.0 uses generator-based CDR (de)serializers under `rosbags.serde.cdr`.
            deser_cache: Dict[str, Any] = {}

            def _get_fields_for_msgtype(msgtype: str) -> Any:
                get_msgdef = getattr(typestore, "get_msgdef", None)
                if callable(get_msgdef):
                    msgdef = get_msgdef(msgtype)
                    fields = getattr(msgdef, "fields", None)
                    if fields is not None:
                        return fields
                    if isinstance(msgdef, tuple) and msgdef:
                        return msgdef[0]
                    return msgdef
                raise KeyError(f"Typestore cannot provide msg definition for: {msgtype}")

            try:
                from rosbags.serde import deserialize_cdr  # type: ignore

                def _deserialize_raw(raw: bytes, msgtype: str) -> Any:
                    return deserialize_cdr(raw, msgtype, typestore=typestore)

            except ImportError:  # rosbags>=0.11
                from rosbags.serde.cdr import generate_deserialize_cdr  # type: ignore

                def _deserialize_raw(raw: bytes, msgtype: str) -> Any:
                    fn = deser_cache.get(msgtype)
                    if fn is None:
                        fields = _get_fields_for_msgtype(msgtype)
                        # ROS 2 bags are CDR little-endian in practice.
                        fn = generate_deserialize_cdr(fields, typestore, "little")
                        deser_cache[msgtype] = fn
                    return fn(raw)

            connections = list(reader.connections)
            for conn, ts_ns, raw in reader.messages(connections=connections):
                topic = conn.topic

                if topic_filter is not None and not topic_filter(topic):
                    continue

                if max_messages_per_topic is not None:
                    if counts.get(topic, 0) >= max_messages_per_topic:
                        continue

                if topic not in out:
                    out[topic] = {
                        "type": getattr(conn, "msgtype", None),
                        "timestamps_ns": [],
                        "messages": [],
                    }

                out[topic]["timestamps_ns"].append(int(ts_ns))

                if decode_messages:
                    try:
                        msg = _deserialize_raw(raw, conn.msgtype)
                        out[topic]["messages"].append(_message_to_builtin(msg))
                    except Exception:
                        # Keep raw bytes if something goes wrong.
                        out[topic]["messages"].append(raw)
                else:
                    out[topic]["messages"].append(raw)

                counts[topic] = counts.get(topic, 0) + 1

        return out

    except Exception as e:  # pragma: no cover
        import traceback
        print(f"rosbags failed to read rosbag with error: {e}\n{traceback.format_exc()}")
        print("rosbags not available or failed to read rosbag; falling back to rosbag2_py (ROS2 stack) for rosbag reading")
        pass

    # Lazy imports so this module still works without ROS2 installed.
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # type: ignore
    except Exception as e:  # pragma: no cover
        raise RuntimeError(
            "Neither 'rosbags' (schema-based) nor 'rosbag2_py' is available; cannot read MCAP rosbag topics"
        ) from e

    if decode_messages:
        try:
            from rclpy.serialization import deserialize_message  # type: ignore
            from rosidl_runtime_py.utilities import get_message  # type: ignore
        except Exception as e:  # pragma: no cover
            raise RuntimeError(
                "rclpy/rosidl_runtime_py not available; cannot deserialize rosbag messages with rosbag2_py"
            ) from e

        try:
            from rosidl_runtime_py.convert import message_to_ordereddict  # type: ignore
        except Exception:  # pragma: no cover
            message_to_ordereddict = None

    metadata_path = bag_dir / "metadata.yaml"
    storage_id: str = "mcap"
    if metadata_path.exists():
        try:
            md = load_yaml(str(metadata_path))
            storage_id = (
                md.get("rosbag2_bagfile_information", {})
                .get("storage_identifier", storage_id)
            )
        except Exception:
            storage_id = "mcap"

    reader = SequentialReader()
    storage_options = StorageOptions(uri=str(bag_dir), storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    type_by_topic = {t.name: t.type for t in topics_and_types}

    out: Dict[str, Dict[str, Any]] = {}
    counts: Dict[str, int] = {}

    while reader.has_next():
        topic, raw, ts_ns = reader.read_next()
        
        # Apply topic filter if provided
        if topic_filter is not None and not topic_filter(topic):
            continue

        if max_messages_per_topic is not None:
            if counts.get(topic, 0) >= max_messages_per_topic:
                continue

        if topic not in out:
            out[topic] = {
                "type": type_by_topic.get(topic),
                "timestamps_ns": [],
                "messages": [],
            }

        out[topic]["timestamps_ns"].append(int(ts_ns))

        if decode_messages:
            msg_type = type_by_topic.get(topic)
            if msg_type is None:
                # Fall back to raw bytes if we cannot map the type.
                out[topic]["messages"].append(raw)
            else:
                msg_cls = get_message(msg_type)
                msg = deserialize_message(raw, msg_cls)
                if message_to_ordereddict is not None:
                    out[topic]["messages"].append(message_to_ordereddict(msg))
                else:
                    out[topic]["messages"].append(msg)
        else:
            out[topic]["messages"].append(raw)

        counts[topic] = counts.get(topic, 0) + 1

    return out


def load_run_data(
    *,
    run_number: Optional[int] = None,
    run_path: Optional[Union[str, os.PathLike]] = None,
    root_dir: Union[str, os.PathLike] = "~/beamng_log_data",
    include_pickle: bool = True,
    include_rosbag: bool = True,
    decode_rosbag_messages: bool = True,
    max_rosbag_messages_per_topic: Optional[int] = None,
    rosbag_pick: str = "latest",
    topic_filter: Optional[callable] = None,
) -> Dict[Union[str, Tuple[Any, Any]], Any]:
    """Load a run into a single consolidated mapping.

    The returned dict keys are:
    - pickle entries: usually (vehicle, sensor) tuples, e.g. ('ego', 'gtstate')
    - rosbag entries: topic name strings, e.g. '/ego/gtstate'
    
    Args:
        run_number: Run number (e.g., 1 for run_001)
        run_path: Direct path to run folder (alternative to run_number)
        root_dir: Root directory containing run folders
        include_pickle: Whether to load pickle data
        include_rosbag: Whether to load rosbag/MCAP data
        decode_rosbag_messages: Whether to deserialize rosbag messages
        max_rosbag_messages_per_topic: Optional limit on messages per topic
        rosbag_pick: Which rosbag to use if multiple exist ('latest' or 'first')
        topic_filter: Optional callable to filter topics (default: None = load all topics)
    
    Returns:
        Dictionary with pickle and rosbag data merged
    """

    run_dir = _resolve_run_path(run_number=run_number, run_path=run_path, root_dir=root_dir)

    merged: Dict[Union[str, Tuple[Any, Any]], Any] = {}

    if include_pickle:
        pkl_data = _load_consolidated_data_from_run_path(run_dir)
        
        # Handle new format with version/format/created_at metadata
        if isinstance(pkl_data, Mapping) and "data" in pkl_data and "version" in pkl_data:
            # New format: extract the nested 'data' dictionary
            actual_data = pkl_data["data"]
            if isinstance(actual_data, Mapping):
                for name, data in actual_data.items():
                    # Prefer (vehicle, sensor) tuple keys if available
                    if isinstance(name, (tuple, list)) and len(name) == 2:
                        merged_name = f"/{name[0]}/{name[1]}"
                        merged[merged_name] = data
                    else:
                        merged[name] = data
            else:
                raise ValueError("Unexpected 'data' format in pickle log data")
        elif isinstance(pkl_data, Mapping):
            # Legacy format: flat dictionary
            for name, data in pkl_data.items():
                # Prefer (vehicle, sensor) tuple keys if available
                if isinstance(name, (tuple, list)) and len(name) == 2:
                    merged_name = f"/{name[0]}/{name[1]}"
                    merged[merged_name] = data
                else:
                    merged[name] = data
        else:
            # Unknown format
            merged[("pickle", "data")] = pkl_data

    if include_rosbag:
        bag_dirs = _find_rosbag_dirs(run_dir)
        if bag_dirs:
            if rosbag_pick == "latest":
                chosen_bag = bag_dirs[-1]
            elif rosbag_pick == "first":
                chosen_bag = bag_dirs[0]
            else:
                raise ValueError("rosbag_pick must be 'latest' or 'first'")

            rosbag_topics = _try_load_rosbag_messages(
                chosen_bag,
                max_messages_per_topic=max_rosbag_messages_per_topic,
                decode_messages=decode_rosbag_messages,
                topic_filter=topic_filter,
            )
            for topic, payload in rosbag_topics.items():
                if topic in merged:
                    print(
                        f"Warning: topic '{topic}' from rosbag conflicts with existing key; skipping."
                    )
                    continue
                # Flatten messages to match pickle format (time series)
                if "messages" in payload and isinstance(payload["messages"], list):
                    flattened_data = _flatten_rosbag_messages(payload["messages"])
                    # Keep timestamps_ns and type, but replace messages with flattened data
                    merged[topic] = {
                        "type": payload.get("type"),
                        "timestamps_ns": payload.get("timestamps_ns", []),
                        **flattened_data  # Merge flattened time series
                    }
                else:
                    merged[topic] = payload

    return merged


def load_log_data(run_number, root_dir="~/beamng_log_data"):
    """
    Load log data and metadata from the specified run number.

    Args:
        run_number (int): The run number (e.g., 1 for run_001).
        root_dir (str): The root directory where run folders are stored (default: ~/beamng_log_data).

    Returns:
        dict: The log data loaded from the run folder.

    Raises:
        FileNotFoundError: If the run folder or log data file does not exist.
    """
    metadata = load_metadata(run_number, root_dir)
    data = load_consolidated_data(run_number, root_dir)
    return metadata, data
