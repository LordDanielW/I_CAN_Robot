import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_simple_nav',
            executable='qwen3_vl_node',
            name='qwen3_vl_node',
            output='screen',
            parameters=[
                {'image_topic': 'camera/image_raw'},
                {'trigger_topic': 'trigger_capture'},
                {'output_topic': 'vlm_output'}
            ]
        )
    ])
