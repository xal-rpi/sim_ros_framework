from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'bng_simulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all YAML config files recursively
        (os.path.join('share', package_name, 'config'), glob('config/**/*.yaml', recursive=True)),
    ],
    install_requires=[
        'setuptools',
        'beamngpy',  # Explicit dependency
        # 'PyYAML',    # For config parsing
        'numpy',      # Commonly needed for sensor data
        'tqdm',       # Progress bar for logging
    ],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Core functionality package for bng_simulator ROS2 interface',
    license='Apache-2.0',
    # entry_points={
    #     'console_scripts': [
    #         'bng_simulator = bng_simulator.simulation_manager:main',
    #     ],
    # },
)