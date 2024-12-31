from setuptools import setup
import os
from glob import glob

package_name = 'fleet_turtlebot4_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f"{package_name}.master", f"{package_name}.slave"],
    data_files=[
        # Include la directory launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Altri file necessari
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # Includi altri file necessari (es. mappe, JSON)
        (os.path.join('share', package_name, 'map'), glob('map/*.json')),
    ],
    install_requires=['setuptools', 'networkx', 'numpy', 'scikit-learn'],
    zip_safe=True,
    maintainer='beniamino',
    maintainer_email='bennibeniamino@gmail.com',
    description='Fleet Turtlebot4 Navigation Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_navigation_node = fleet_turtlebot4_navigation.master.master_navigation_node:main',
            'simulated_slave_navigation_node = fleet_turtlebot4_navigation.slave.simulated_slave_navigation_node:main',
            'slave_navigation_node = fleet_turtlebot4_navigation.slave.slave_navigation_node:main',
            'publish_waypoints = fleet_turtlebot4_navigation.publish_waypoints:main'
        ],
    },
)
