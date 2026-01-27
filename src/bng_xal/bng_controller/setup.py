from setuptools import find_packages, setup, Extension
import os
from glob import glob

package_name = "bng_controller"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="comejv",
    maintainer_email="vincec4@rpi.edu",
    description="High-level controller for BeamNG simulation",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "run_controller = bng_controller.controller_interface:main",
            "high_level_controller = bng_controller.high_level_controller:main",
            "gt_state_bridge = bng_controller.gt_state_bridge:main",
            "path_viz = bng_controller.path_viz:main",
            "generate_circle_path = bng_controller.scripts.generate_circle_path:main",
            "generate_path = bng_controller.scripts.generate_path:main",
            "send_override_target = bng_controller.scripts.send_override_target:main",
        ],
    },
)
