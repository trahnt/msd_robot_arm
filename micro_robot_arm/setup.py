import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'micro_robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/config.rviz')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben, Matthew',
    maintainer_email='benlagreca02@gmail.com, mjh9585@rit.edu',
    description='Simple package for running micro robot arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piJointListener = micro_robot_arm.piNode:main'
        ],
    },
)
