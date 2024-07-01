from setuptools import setup

package_name = 'autoware_fms_offline_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mechanopixel',
    maintainer_email='annie@mechanopixel.xyz',
    description='Automatic goal publisher. Emulation of FMS.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = autoware_fms_offline_gui.main:main',
        ],
    },
)
