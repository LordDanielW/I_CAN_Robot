#!/usr/bin/env python3
"""
Remote Nodes Launch File
Runs CPU-intensive nodes on a remote/powerful computer:
- Vosk speech recognition server
- Other processing-intensive nodes can be added here
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Vosk server - processes audio stream for speech recognition
        Node(
            package='ican_voice',
            executable='vosk_server_node',
            name='vosk_server',
            output='screen'
        ),
        
        # Add other CPU-intensive nodes here as needed
    ])
