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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='otoshuki',
    maintainer_email='guining.pertin.iitg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'simulator_node = mopat_pkg.simulator_node:main',
            'camera_node = mopat_pkg.camera_node:main',
            'occ_map_node = mopat_pkg.occ_map_node:main',
        ],
    },
)
