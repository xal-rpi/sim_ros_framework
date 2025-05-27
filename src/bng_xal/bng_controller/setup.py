from setuptools import find_packages, setup, Extension
import os
from glob import glob

package_name = "bng_controller"

c_extensions = []
# The glob pattern is relative to this setup.py file.
# Given setup.py is at src/bng_xal/bng_controller/setup.py,
# "bng_controller/core/*.c" will look for C files in
# src/bng_xal/bng_controller/bng_controller/core/
c_source_files = glob("bng_controller/core/*.c")

for c_file in c_source_files:
    module_path_no_ext = os.path.splitext(c_file)[0]
    module_name = module_path_no_ext.replace(os.sep, ".")

    c_extensions.append(
        Extension(
            module_name,
            sources=[c_file],
            extra_compile_args=["-O3"],
        )
    )

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "paths"), glob("resource/paths/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="comev",
    maintainer_email="vincec4@rpi.edu",
    description="High-level controller for BeamNG simulation",
    license="TODO: License declaration",
    ext_modules=c_extensions,
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "run_controller = bng_controller.controller_interface:main",
            "high_level_controller = bng_controller.high_level_controller:main",
            "path_viz = bng_controller.path_viz:main",
            "generate_circle_path = bng_controller.scripts.generate_circle_path:main",
            "generate_path = bng_controller.scripts.generate_path:main",
            "send_override_target = bng_controller.scripts.send_override_target:main",
        ],
    },
)
