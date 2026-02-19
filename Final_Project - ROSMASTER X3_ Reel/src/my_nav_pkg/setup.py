#!/usr/bin/env python3
from setuptools import setup
from glob import glob
import os

package_name = 'my_nav_pkg'

setup(
    name=package_name,
    version='0.1.0',
    # Correction : on liste les packages explicitement ou via find_packages
    packages=[package_name], 
    data_files=[
        # Index ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files (Modifié pour capturer .py et .launch.py)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + glob('launch/*.py')),

        # Config files (nav2_params.yaml, etc.)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        
        # Maps files (map_win.yaml et map_win.pgm)
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        
        # RViz configs (trc.rviz)
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='billy',
    maintainer_email='billy@todo.todo',
    description='Navigation, SLAM and localization package for X3 robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'algo_win = my_nav_pkg.algo_win:main',
        ],
    },
)
