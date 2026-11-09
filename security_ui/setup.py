import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'security_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ljs',
    maintainer_email='glorybell1984@gmail.com',
    description='Web dashboard served through rosbridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [

        ],
    },
)
