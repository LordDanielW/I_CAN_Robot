#!/usr/bin/env python3
"""
Local Nodes Launch File
Runs lightweight nodes on the local computer:
- Camera image publisher (5Hz)
- Audio streamer (streams microphone audio)
- YOLO object detection (YOLOv8/v13, processes camera feed)
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
                'device_id': 0,  # /dev/video0 (Insta360 X5)
            }],
            remappings=[
                ('image', '/camera/image_raw'),
            ],
            output='screen'
        ),
        
        # Audio streamer - captures and streams microphone audio
        Node(
            package='ican_voice',
            executable='audio_streamer_node',
            name='audio_streamer',
            output='screen'
        ),        

        # Audio playback - captures and plays speaker audio
        Node(
            package='ican_voice',
            executable='audio_playback_node',
            name='audio_playback',
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
