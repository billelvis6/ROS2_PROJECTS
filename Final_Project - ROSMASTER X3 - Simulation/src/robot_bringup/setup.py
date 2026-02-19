from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='billy',
    maintainer_email='billy@example.com',
    description='Bringup package for TRC2025 robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # si tu as des scripts à exécuter directement
        ],
    },
    data_files=[
        # Package resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package XML
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
)

