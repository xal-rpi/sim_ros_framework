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

    _config_path = None
    _config = None
    _lock = Lock()

    # Base directories (populated on first access)
    _CONFIG_DIR: Optional[str] = None
    _SCENARIO_DIR: Optional[str] = None
    _VEHICLES_DIR: Optional[str] = None

    @classmethod
    def is_ready(cls):
        return cls._config is not None

    @classmethod
    def _init_dirs(cls) -> None:
        """Resolve and cache CONFIG_DIR, SCENARIO_DIR, VEHICLES_DIR."""
        if cls._CONFIG_DIR is not None:
            return

        base = get_package_share_directory("bng_simulator")

        cls._CONFIG_DIR = os.path.join(base, "config")
        cls._SCENARIO_DIR = os.path.join(base, "scenarios")
        cls._VEHICLES_DIR = os.path.join(base, "vehicles")

    @classmethod
    def get_config(cls, filename_or_path: Optional[str]) -> Dict[str, Any] | None:
        """
        Load & cache a YAML by absolute path or by filename (searched in CONFIG_DIR,
        SCENARIO_DIR, VEHICLES_DIR). Returns parsed dict or error.

        Args:
            filename_or_path: either an absolute/relative path or just "my.yaml"

        Returns:
            Parsed YAML dict or None.
        """
        if cls._config_path is not None:
            return cls._config
        if filename_or_path is None:
            raise FileNotFoundError(
                "Trying to get_config for the first time without filename_or_path"
            )

        cls._init_dirs()

        # If given an explicit path, try it first:
        candidates = [filename_or_path]

        # If it's not absolute, also try in our known directories
        if not os.path.isabs(filename_or_path):
            for d in (cls._CONFIG_DIR, cls._SCENARIO_DIR, cls._VEHICLES_DIR):
                candidates.append(os.path.join(d, filename_or_path))

        for path in candidates:
            abs_path = os.path.abspath(path)

            if os.path.isfile(abs_path):
                # thread‐safe load + cache
                with cls._lock:
                    # re-check inside lock
                    if cls._config_path is not None:
                        if abs_path == cls._config_path:
                            return cls._config
                        raise RuntimeError(
                            f"Trying to load two different config files : {abs_path} and {cls._config}"
                        )
                    else:
                        cls._config_path = abs_path
                        cls._config = load_yaml(cls._config_path)
                print(f"Got config from {abs_path}")
                return cls._config

        raise FileNotFoundError(
            f"Config file '{filename_or_path}' not found in: {candidates}"
        )
