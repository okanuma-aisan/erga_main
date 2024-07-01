from glob import glob
import os
from setuptools import setup

package_name = 'autoware_fms_status'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mechanopixel',
    maintainer_email='annie@mechanopixel.xyz',
    description='Node that obtains information from FMS API and publishes it as ROS2 messages.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = autoware_fms_status.main:main',
        ],
    },
)
