import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ekf_localization_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'models', 'turtlebot3_burger_gps'),
            glob(os.path.join('models', 'turtlebot3_burger_gps', '*'))),
        (os.path.join('share', package_name, 'scripts'),
            glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lahiru',
    maintainer_email='lahirukanishka2000@gmail.com',
    description='EKF Localization for TurtleBot3 using GPS and IMU',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_localization_node = ekf_localization_tb3.ekf_node:main',
            'data_logger = ekf_localization_tb3.data_logger:main',
        ],
    },
)
