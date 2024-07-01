import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dummy_expect_rough_rois_pub'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer_email='kanaoka@novetec.co.jp',
    description='Dummy expect_rois and rough_rois publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_expect_rough_rois_pub_node = dummy_expect_rough_rois_pub.dummy_expect_rough_rois_pub_node:main'
        ],
    },
)
