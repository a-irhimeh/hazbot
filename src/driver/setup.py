from setuptools import setup
import os
from glob import glob

package_name = 'driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Differential drive robot driver package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Motor controller node
            'motor_controller = driver.motor_controller:main',
            'm1 = driver.m1_controller:main',
            # Odometry node
            'odometry = driver.odometry:main',
        ],
    },
)