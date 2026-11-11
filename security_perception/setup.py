import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'security_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ljs',
    maintainer_email='glorybell1984@gmail.com',
    description='Perception: ArUco detection and object position in the map frame',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception = security_perception.perception_node:main',
            'make_markers = security_perception.make_markers:main',
            'camera_publisher = security_perception.camera_publisher:main',
            'find_dictionary = security_perception.find_dictionary:main'
        ],
    },
)
