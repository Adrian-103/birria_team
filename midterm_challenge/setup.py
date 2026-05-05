from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'midterm_challenge'

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
            'vision_exe = midterm_challenge.vision:main',
            'pid_exe = midterm_challenge.point_PID_controller:main',
            'path_generator_exe = midterm_challenge.path_generator:main',
            'path_generator_c_exe = midterm_challenge.path_generator_c:main',
        ],
    },
)
