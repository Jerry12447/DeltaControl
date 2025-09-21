from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'yolo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'),
         glob('models/*.pt'))
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Jerry12447',
    maintainer_email='c111104231@mkust.edu.tw',
    description='YOLO Object Detection Package for Delta Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yoloboundingbox = yolo.yoloboundingbox:main',
        ],
    },
)
