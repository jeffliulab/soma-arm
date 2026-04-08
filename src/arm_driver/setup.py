from setuptools import find_packages, setup
from glob import glob

package_name = 'arm_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Jeff Liu Lab',
    maintainer_email='jeff@jeffliulab.com',
    description='USB serial driver for Waveshare RoArm-M2-S, exposed as a ROS 2 node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_driver_node = arm_driver.arm_driver_node:main',
            'moveit_bridge_node = arm_driver.moveit_bridge_node:main',
            'probe = arm_driver.roarm_protocol:_cli_probe',
        ],
    },
)
