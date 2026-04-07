"""Launch robot model in RViz for visualization and joint testing."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('arm_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'soma_robot.urdf.xacro')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # Publish robot URDF to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # GUI slider to move joints manually
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'display.rviz')],
        ),
    ])
