import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'A_star'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
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
            'global_planner = A_star.global_planner:main',
            'path_follower = A_star.path_follower:main',
            'odometry = A_star.odometry_node:main',
        ],
    },
)
