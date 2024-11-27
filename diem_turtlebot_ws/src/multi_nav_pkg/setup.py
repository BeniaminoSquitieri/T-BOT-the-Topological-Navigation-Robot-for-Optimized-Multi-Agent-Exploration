from setuptools import setup
import os
from glob import glob

package_name = 'multi_nav_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'multi_nav_pkg.master_navigation_node',
        'multi_nav_pkg.backup_master_navigation_node',
        'multi_nav_pkg.slave_navigation_node',
        'multi_nav_pkg.graph_partitioning'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Includi i file di lancio
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'networkx'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Master-Slave Navigation Package for Multi-TurtleBot4',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_navigation_node = multi_nav_pkg.master_navigation_node:main',
            'backup_master_navigation_node = multi_nav_pkg.backup_master_navigation_node:main',
            'slave_navigation_node = multi_nav_pkg.slave_navigation_node:main',
        ],
    },
)
