import os
from glob import glob
from setuptools import setup

package_name = 'mopat_pkg'

setup(
    name=package_name,
    version='5.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        ('lib/'+package_name, [package_name+'/test.png'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='MoPAT Team',
    author_email='guining.pertin.iitg@gmail.com',
    maintainer='MoPAT Team',
    maintainer_email='guining.pertin.iitg@gmail.com',
    description='ROS2 Package for MoPAT system',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = mopat_pkg.simulator_node:main',
            'camera_node = mopat_pkg.camera_node:main',
            'occ_map_node = mopat_pkg.occ_map_node:main',
            'config_space_node = mopat_pkg.config_space_node:main',
            'discretization_node = mopat_pkg.discretization_node:main',
            'plot_node = mopat_pkg.plot_node:main',
            'motion_planning_node = mopat_pkg.motion_planner_node:main',
            'mopat_astar_ros = mopat_pkg.mopat_astar_ros:main'
        ],
    },
)
