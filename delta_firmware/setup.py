from setuptools import setup
import os
from glob import glob

package_name = 'delta_firmware'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'cv-bridge'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Delta Robot Control Package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_plan = delta_firmware.Trajectory_plan:main',
            'delta_firmware_control = delta_firmware.delta_firmware_control:main',
            'delta_firmware_api = delta_firmware.delta_firmware_api:main'
        ],
    },
)