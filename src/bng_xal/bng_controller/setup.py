from setuptools import find_packages, setup, Extension
import os
from glob import glob

package_name = "bng_controller"

# Define the C extension module
controller_core_module = Extension(
    "bng_controller.core.controller_core",
    sources=["bng_controller/core/controller_core.c"],
    extra_compile_args=["-O3"],  # Optimization flag
)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="comev",
    maintainer_email="44554692+comejv@users.noreply.github.com",
    description="High-level controller for BeamNG simulation",
    license="TODO: License declaration",
    ext_modules=[controller_core_module],
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "run_controller = bng_controller.controller_interface:main",
            "high_level_controller = bng_controller.high_level_controller:main",  # Will be started by run_controller
        ],
    },
)
