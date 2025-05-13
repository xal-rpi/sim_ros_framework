import os
from threading import Lock
from typing import Any, Dict, Optional

from ament_index_python.packages import get_package_share_directory
from bng_simulator.utils.io_dict_utils import load_yaml


class ConfigManager:
    """
    Singleton‐style manager for loading and caching YAML configuration files.
    It also centralizes resolution of CONFIG_DIR, SCENARIO_DIR, and VEHICLES_DIR.
    """

    # Cache: absolute_path -> parsed dict
    _cache: Dict[str, Dict[str, Any]] = {}
    _lock = Lock()

    # Base directories (populated on first access)
    _CONFIG_DIR: Optional[str] = None
    _SCENARIO_DIR: Optional[str] = None
    _VEHICLES_DIR: Optional[str] = None

    @classmethod
    def _init_dirs(cls) -> None:
        """Resolve and cache CONFIG_DIR, SCENARIO_DIR, VEHICLES_DIR."""
        if cls._CONFIG_DIR is not None:
            return

        # Try via ROS package
        try:
            base = get_package_share_directory("bng_simulator")
        except Exception:
            # Fallback to source layout (this file's parent/../config)
            here = os.path.dirname(os.path.abspath(__file__))
            base = os.path.normpath(os.path.join(here, "../config"))

        cls._CONFIG_DIR = os.path.join(base, "config")
        cls._SCENARIO_DIR = os.path.join(base, "scenarios")
        cls._VEHICLES_DIR = os.path.join(base, "vehicles")

    @classmethod
    def get_config(cls, filename_or_path: str) -> Dict[str, Any]:
        """
        Load & cache a YAML by absolute path or by filename (searched in CONFIG_DIR,
        SCENARIO_DIR, VEHICLES_DIR). Returns parsed dict or error.

        Args:
            filename_or_path: either an absolute/relative path or just "my.yaml"

        Returns:
            Parsed YAML dict or None.
        """
        cls._init_dirs()

        # If given an explicit path, try it first:
        candidates = [filename_or_path]

        # If it's not absolute, also try in our known directories
        if not os.path.isabs(filename_or_path):
            for d in (cls._CONFIG_DIR, cls._SCENARIO_DIR, cls._VEHICLES_DIR):
                candidates.append(os.path.join(d, filename_or_path))

        for path in candidates:
            abs_path = os.path.abspath(path)
            # If cached, return immediately
            if abs_path in cls._cache:
                return cls._cache[abs_path]

            if os.path.isfile(abs_path):
                # thread‐safe load + cache
                with cls._lock:
                    # re-check inside lock
                    if abs_path not in cls._cache:
                        data = load_yaml(abs_path)
                        cls._cache[abs_path] = data
                print(f"Got config from {abs_path}")
                return cls._cache[abs_path]

        raise FileNotFoundError(
            f"Config file '{filename_or_path}' not found in: {candidates}"
        )

    @classmethod
    def clear_cache(cls, filename_or_path: Optional[str] = None) -> None:
        """
        Clear the cache for a single file or the entire cache.

        Args:
            filename_or_path: if provided, only that entry is removed.
        """
        if filename_or_path:
            abs_path = os.path.abspath(filename_or_path)
            with cls._lock:
                cls._cache.pop(abs_path, None)
        else:
            with cls._lock:
                cls._cache.clear()
