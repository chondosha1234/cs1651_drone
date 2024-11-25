from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'mavsdk',
    ],
    zip_safe=True,
    maintainer='chondosha',
    maintainer_email='jmiller61193@gmail.com',
    description='ROS2 tutorial and practice',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'drone_publisher = my_package.drone_test:main',
            'mavsdk_node = my_package.mavsdk_test:main',
        ],
    },
)
