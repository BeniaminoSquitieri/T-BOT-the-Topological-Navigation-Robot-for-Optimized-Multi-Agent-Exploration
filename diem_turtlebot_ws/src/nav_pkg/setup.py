from setuptools import find_packages, setup

package_name = 'nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gdesimone',
    maintainer_email='44608428+gdesimone97@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            "simple_nav = nav_pkg.simple_nav:main",
            "nav_step = nav_pkg.nav_step:main",
            "obj_avoidance = nav_pkg.obstacle_avoidance:main",
            "odom = nav_pkg.odom:main",
            "navigation = nav_pkg.navigation_phase.navigation:main",
        ],
    },
)
