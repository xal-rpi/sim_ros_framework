from pathlib import Path
import xml.etree.ElementTree as ET

from setuptools import find_packages, setup

package_name = "bng_simulator"


def package_version() -> str:
    tree = ET.parse(Path(__file__).parent / "package.xml")
    return tree.findtext("version").strip()


setup(
    name=package_name,
    version=package_version(),
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="franck",
    maintainer_email="djeumf2@rpi.edu",
    description="BeamNG xlab simulation manager and scenario compose",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "sim_manager_node = bng_simulator.sim_manager_node:main",
            "sim_shell = bng_simulator.scripts.sim_shell:main",
            "sim_control = bng_simulator.scripts.sim_control:main",
            "start_logs = bng_simulator.scripts.start_logs:main",
            "find_ema = bng_simulator.scripts.find_test_ema_gtstate:main",
        ],
    },
)
