from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'xtgt_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'xtgt_bringup'), glob('xtgt_bringup/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xtgt',
    maintainer_email='KTKIM@kinetix.co.kr',
    description='XTGT ROS2 ROBOT BRINGUP FILE',
    license='Copyright 2026. Kinectix Co. All rights reserved.',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_filter = xtgt_bringup.lidar_filter:main',
            'dual_rtk_yaw = xtgt_bringup.dual_rtk_yaw:main'
        ],
    },
)
