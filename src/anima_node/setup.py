from setuptools import find_packages, setup

package_name = 'anima_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/skills.yaml']),
        ('share/' + package_name + '/launch', ['launch/anima.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeff Liu Lab',
    maintainer_email='contact@jeffliulab.com',
    description='ANIMA cognitive framework ROS 2 wrapper for SmartRobotArm',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'anima_core = anima_node.nodes.anima_core_node:main',
            'skill_executor = anima_node.nodes.skill_executor_node:main',
        ],
    },
)
