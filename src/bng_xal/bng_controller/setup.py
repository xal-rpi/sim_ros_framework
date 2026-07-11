from pathlib import Path
import xml.etree.ElementTree as ET

from setuptools import find_packages, setup

package_name = "bng_controller"


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
    maintainer="comejv",
    maintainer_email="vincec4@rpi.edu",
    description="BeamNG xlab companion I/O bridge (sensor_dispatcher, VehicleSession)",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "sensor_dispatcher = bng_controller.sensor_dispatcher:main",
            "test_vehicle_udp = bng_controller.scripts.test_vehicle_udp:main",
            "test_vehicle_sensor_udp = bng_controller.scripts.test_vehicle_sensor_udp:main",
            "test_llc_wr_torque = bng_controller.scripts.test_llc_wr_torque:main",
        ],
    },
)
