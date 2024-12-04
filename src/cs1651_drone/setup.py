from setuptools import find_packages, setup

package_name = 'cs1651_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjm267',
    maintainer_email='jjm267@pitt.edu',
    description='CS1651 Class Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = cs1651_drone.camera_node:main',
            'drone_listener = cs1651_drone.drone_listener:main',
            'drone_node = cs1651_drone.drone_node:main',
            'drone_takeoff_test = cs1651_drone.drone_takeoff_test:main',
        ],
    },
)
