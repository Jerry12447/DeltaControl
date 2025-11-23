from setuptools import setup
import os
from glob import glob

package_name = 'plant_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=['plant_detection'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'model'), glob('model/*.pt')),
    ],
    install_requires=['setuptools', 'scipy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='yolo辨識整合疏苗算法',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plant_detection_node = plant_detection.plant_detection:main',
            'plant_detection_video_node = plant_detection.plant_detection_video:main',
        ],
    },
)
