import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'soa_rosetta'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'contracts'),
            glob('contracts/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='42076119+htchr@users.noreply.github.com',
    description='Config files and scripts for using rosetta packages with LeRobot policies on the SOA robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'soa_action_relay_node = soa_rosetta.soa_action_relay_node:main',
        ],
    },
)
