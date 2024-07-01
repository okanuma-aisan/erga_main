from setuptools import setup

package_name = 'mobileye_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sonjunseong',
    maintainer_email='sonjunseong@todo.todo',
    description='The mobileyetest package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobileye_node = mobileye_node.mobileye_node:main',
        ],
    },
)
