# resource_manager.py

import os
from typing import Optional
from ament_index_python.packages import get_package_share_directory


class ResourceManager:
    """
    Locate and (optionally) load files from the share/<package> folder.
    """

    @staticmethod
    def get_path(package: str, *subpaths: str) -> str:
        """
        Return the absolute path to share/<package>/subpaths[0]/.../subpaths[-1].
        """
        base = get_package_share_directory(package)
        return os.path.join(base, *subpaths)

    @staticmethod
    def exists(package: str, *subpaths: str) -> bool:
        """
        Check whether the given file exists under share/<package>/...
        """
        return os.path.exists(ResourceManager.get_path(package, *subpaths))

    @staticmethod
    def load_text(
        package: str, *subpaths: str, encoding: Optional[str] = "utf-8"
    ) -> str:
        """
        Return the file’s contents as str.
        """
        fn = ResourceManager.get_path(package, *subpaths)
        with open(fn, "r", encoding=encoding) as f:
            return f.read()

    @staticmethod
    def load_bytes(package: str, *subpaths: str) -> bytes:
        """
        Return the file’s contents as bytes.
        """
        fn = ResourceManager.get_path(package, *subpaths)
        with open(fn, "rb") as f:
            return f.read()
