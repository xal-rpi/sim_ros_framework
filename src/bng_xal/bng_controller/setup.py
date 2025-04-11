from setuptools import find_packages, setup
import os
from glob import glob

package_name = "bng_controller"

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
    zip_safe=True,
    maintainer="comev",
    maintainer_email="44554692+comejv@users.noreply.github.com",
    description="High-level controller for BeamNG simulation",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "run_controller = bng_controller.controller_interface:main",
        ],
    },
)
