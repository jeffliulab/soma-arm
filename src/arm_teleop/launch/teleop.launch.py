"""One-shot teleop launch: start arm_driver + gamepad_teleop together.

Usage:
    ros2 launch arm_teleop teleop.launch.py
    ros2 launch arm_teleop teleop.launch.py serial_port:=/dev/ttyACM0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        DeclareLaunchArgument("publish_rate", default_value="20.0"),

        Node(
            package="arm_driver",
            executable="arm_driver_node",
            name="arm_driver",
            output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": LaunchConfiguration("baud_rate"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            }],
        ),
        Node(
            package="arm_teleop",
            executable="gamepad_teleop_node",
            name="gamepad_teleop",
            output="screen",
        ),
    ])
