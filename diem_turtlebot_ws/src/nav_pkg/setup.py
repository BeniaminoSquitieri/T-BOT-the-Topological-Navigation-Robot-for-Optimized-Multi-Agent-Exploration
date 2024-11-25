from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Includi i file di launch e map se necessari
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'map'), glob('nav_pkg/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gdesimone',
    maintainer_email='44608428+gdesimone97@users.noreply.github.com',
    description='Robot Navigation Node for Multi-Robot System',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            "navigation = nav_pkg.navigation:main",
            "robot_navigation_node = nav_pkg.robot_navigation_node:main",
            "publish_waypoints = nav_pkg.publish_waypoints:main",
        ],
    },
)
