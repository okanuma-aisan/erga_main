from glob import glob
import os
from setuptools import setup

package_name = 'auto_csv_logger'

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
    description='Handy ROS2 node for automatically logging arbitrary messages into a CSV file.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = auto_csv_logger.main:main',
        ],
    },
)
