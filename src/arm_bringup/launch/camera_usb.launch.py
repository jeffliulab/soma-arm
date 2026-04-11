"""Project-local usb_cam launch for the Logitech C922.

Why this exists:
- the system usb_cam launch currently imports a pydantic-based helper that
  breaks inside our `.venv`
- we want stable project-local defaults and topic names
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("arm_bringup"))
    default_params = bringup_share / "config" / "c922_usb_cam.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("framerate", default_value="30.0"),
        DeclareLaunchArgument("image_width", default_value="1280"),
        DeclareLaunchArgument("image_height", default_value="720"),
        DeclareLaunchArgument("pixel_format", default_value="mjpeg2rgb"),
        DeclareLaunchArgument("io_method", default_value="mmap"),
        DeclareLaunchArgument("frame_id", default_value="camera"),
        DeclareLaunchArgument("params_file", default_value=str(default_params)),

        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "video_device": LaunchConfiguration("video_device"),
                    "framerate": LaunchConfiguration("framerate"),
                    "image_width": LaunchConfiguration("image_width"),
                    "image_height": LaunchConfiguration("image_height"),
                    "pixel_format": LaunchConfiguration("pixel_format"),
                    "io_method": LaunchConfiguration("io_method"),
                    "frame_id": LaunchConfiguration("frame_id"),
                },
            ],
            remappings=[
                ("image_raw", "/camera/image_raw"),
                ("image_raw/compressed", "/camera/image_raw/compressed"),
                ("image_raw/compressedDepth", "/camera/image_raw/compressedDepth"),
                ("image_raw/theora", "/camera/image_raw/theora"),
                ("camera_info", "/camera/camera_info"),
            ],
        ),
    ])
