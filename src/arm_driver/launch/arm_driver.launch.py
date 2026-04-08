"""Launch just the RoArm-M2-S driver node (no teleop, no MoveIt)."""

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
    ])
