from setuptools import setup

package_name = 'multi_robot_pkg_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_navigation_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Package for multi-robot navigation',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
