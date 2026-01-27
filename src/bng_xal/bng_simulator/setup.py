from setuptools import setup, find_packages

package_name = "bng_simulator"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "beamngpy",  # Explicit dependency
        "PyYAML",  # For config parsing
        "numpy",  # Commonly needed for sensor data
        "tqdm",  # Progress bar for logging
    ],
    zip_safe=True,
    maintainer="franck",
    maintainer_email="djeumf2@rpi.edu",
    extras_require={
        "test": ["pytest"],
    },
    description="Core functionality package for bng_simulator ROS2 interface",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sim_manager_node = bng_simulator.sim_manager_node:main",
            "sim_shell = bng_simulator.scripts.sim_shell:main",
            "sim_control = bng_simulator.scripts.sim_control:main",
            "start_logs = bng_simulator.scripts.start_logs:main",
            "find_ema = bng_simulator.scripts.find_test_ema_gtstate:main"
        ],
    },
)
