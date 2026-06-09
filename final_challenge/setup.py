from setuptools import find_packages, setup

package_name = 'final_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params', [
            'params/line_follower_params.yaml',
            'params/line_pid_controller_params.yaml',
            'params/pid_test_params.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/line_follower.launch.py',
            'launch/final_challenge.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='a01665899@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_follower_cv = final_challenge.line_follower_cv:main',
            'camera_node = final_challenge.camera_node:main',
            'pid_test_node = final_challenge.pid_test_node:main',
            'line_pid_controller = final_challenge.line_pid_controller:main',
            'intersection = final_challenge.zebra_detector:main',
            'fuzzy_vel = final_challenge.cont_vel:main',
            'control_master = final_challenge.control_master:main',
            'odometry_node = final_challenge.odometry_node:main',
            'linea_amarilla = final_challenge.caution_detector:main',
        ],
    },
)
