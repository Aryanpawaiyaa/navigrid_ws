import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'navigrid_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aryan',
    maintainer_email='aryan@example.com',
    description='Adaptive cost-aware path planner, local reactive controller, and velocity-dependent safety override for NaviGrid AMR',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'costmap_generator = navigrid_navigation.costmap_generator:main',
            'adaptive_global_planner = navigrid_navigation.adaptive_global_planner:main',
            'local_reactive_controller = navigrid_navigation.local_reactive_controller:main',
            'safety_override_node = navigrid_navigation.safety_override_node:main',
        ],
    },
)
