from setuptools import setup
import os
from glob import glob

package_name = 'delta_robot_isaacsim'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Delta Robot Control Package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_plan = delta_robot_isaacsim.Trajectory_plan:main',
            'delta_control = delta_robot_isaacsim.delta_control:main',
        ],
    },
)