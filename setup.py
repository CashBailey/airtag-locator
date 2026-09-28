from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'stacy_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'PyQt5',
    ],
    zip_safe=True,
    maintainer='Cash Bailey',
    maintainer_email='110917999+CashBailey@users.noreply.github.com',
    description='ROS 2 gimbal robot that aims a directional antenna to locate hidden Bluetooth trackers.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = stacy_py.bridge:main',
            'bridge_send = stacy_py.bridge_send:main',
            'strongestMAC = stacy_py.strongestMAC:main',
            'robot_control_gui = stacy_py.robot_control_gui:main',
            'horizontal_scan_node = stacy_py.horizontal_scan_node:main',
            'vertical_scan_node = stacy_py.vertical_scan_node:main'
        ],
    },
)
