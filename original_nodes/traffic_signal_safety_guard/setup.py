import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'traffic_signal_safety_guard'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sit',
    maintainer_email='kanaoka@novatec.co.jp',
    description='Traffic signal safety guard',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_signal_safety_guard_node = traffic_signal_safety_guard.traffic_signal_safety_guard_node:main'
        ],
    },
)
