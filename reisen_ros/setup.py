from setuptools import find_packages, setup

package_name = 'reisen_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luu',
    maintainer_email='asdzxc87225@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'talker = reisen_ros.publisher_member_function:main',
             'control = reisen_ros.control_node:main',
             'odom = reisen_ros.diff_odom:main',
             'serial = reisen_ros.serial_node:main',
        ],
    },
)
