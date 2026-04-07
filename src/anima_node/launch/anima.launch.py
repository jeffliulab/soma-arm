"""Launch ANIMA cognitive framework nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'llm_backend',
            default_value='mock',
            description='LLM backend: mock, anthropic, openai',
        ),

        # ANIMA Core (NLU + Planning)
        Node(
            package='anima_node',
            executable='anima_core',
            name='anima_core',
            parameters=[{
                'llm_backend': LaunchConfiguration('llm_backend'),
                'llm_model': 'claude-opus-4-6',
            }],
            output='screen',
        ),

        # Skill Executor
        Node(
            package='anima_node',
            executable='skill_executor',
            name='skill_executor',
            output='screen',
        ),
    ])
