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
        ('lib/'+package_name+'/data', [package_name+'/data/mask_parameters']),
        ('lib/'+package_name+'/data', [package_name+'/data/test.png']),
        ('lib/'+package_name+'/algorithms', [package_name+'/algorithms/mopat_astar_ros.py']),
        ('lib/'+package_name+'/algorithms', [package_name+'/algorithms/detect_blob.py']),
        ('lib/'+package_name+'/algorithms', [package_name+'/algorithms/localize.py']),
        ('lib/'+package_name+'/algorithms', [package_name+'/algorithms/import_mask_params.py']),
        ('lib/'+package_name+'/algorithms', [package_name+'/algorithms/videocap.py']),
        ('lib/'+package_name+'/algorithms', [package_name+'/algorithms/mopat_lib.py']),
        ('lib/'+package_name+'/static', [package_name+'/static/bootstrap.min.css']),
        ('lib/'+package_name+'/static', [package_name+'/static/bootstrap.min.js']),
        ('lib/'+package_name+'/static', [package_name+'/static/jquery.js']),
        ('lib/'+package_name+'/static', [package_name+'/static/jquery.min.js']),
        ('lib/'+package_name+'/static', [package_name+'/static/myscript.js']),
        ('lib/'+package_name+'/static', [package_name+'/static/small-business.css']),
        ('lib/'+package_name+'/static', [package_name+'/static/style.css']),
        ('lib/'+package_name+'/templates', [package_name+'/templates/index.html']),
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
            'simulator_node = mopat_pkg.simulator_node_new:main',
            'camera_node = mopat_pkg.camera_node:main',
            'localization_node = mopat_pkg.localization_node:main',
            'occ_map_node = mopat_pkg.occ_map_node:main',
            'config_space_node = mopat_pkg.config_space_node:main',
            'discretization_node = mopat_pkg.discretization_node:main',
            'plot_node = mopat_pkg.plot_node:main',
            'motion_planning_node = mopat_pkg.motion_planner_node:main',
            'stream_node = mopat_pkg.stream_node:main',
        ],
    },
)
