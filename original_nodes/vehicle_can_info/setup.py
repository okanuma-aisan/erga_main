import os
from glob import glob
from setuptools import setup

package_name = 'vehicle_can_info'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma][xml]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MechanoPixel',
    maintainer_email='mechanopixel@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "main = vehicle_can_info.vehicle_info_publisher:main"
        ],
    },
)
