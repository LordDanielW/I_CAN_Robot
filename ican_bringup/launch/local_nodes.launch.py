#!/usr/bin/env python3
"""
Local Nodes Launch File
Runs lightweight nodes on the local computer:
- Camera image publisher (5Hz)
- Audio streamer (streams microphone audio)
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera node - publishes images at 5Hz
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            parameters=[{
                'frequency': 5.0,  # Publish at 5Hz
            }],
            output='screen'
        ),
        
        # Audio streamer - captures and streams microphone audio
        Node(
            package='ican_voice',
            executable='audio_streamer_node',
            name='audio_streamer',
            output='screen'
        ),
        
        # Joystick node - publishes joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
    ])
