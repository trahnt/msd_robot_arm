from setuptools import find_packages, setup
from glob import glob

package_name = 'arm_pkg'

setup(
    name=package_name,
    version='0.6.9',
    packages=find_packages(exclude=['test']),
    # add other files to 'install' here
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name+'/urdf/', glob('urdf/*')),
        ('share/' + package_name+'/config/', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='benlagreca02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
