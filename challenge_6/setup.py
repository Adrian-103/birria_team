from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'challenge_6'

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
    maintainer='adrian',
    maintainer_email='adr.dlp@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = challenge_6.camera_node:main',
            'line_node = challenge_6.line_node:main',
            'hough_node = challenge_6.hough_node:main',
            'vision = challenge_6.vision:main',
            'odometry_node = challenge_6.odometry_node:main',
            'pid_challenge6 = challenge_6.pid_challenge6:main'
        ],
    },
)
