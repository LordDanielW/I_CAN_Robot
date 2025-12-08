#!/usr/bin/env python3
"""
I_CAN Voice Remote Launch File
Launches the speech recognition node for voice control.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Whisper Server - Speech recognition (converts audio to text)
        Node(
            package='ican_voice',
            executable='whisper_server_node',
            name='whisper_server',
            output='screen',
            parameters=[{
                'model_size': 'base.en',
                'device': 'cpu',
                'compute_type': 'int8'
            }]
        ),
    ])
