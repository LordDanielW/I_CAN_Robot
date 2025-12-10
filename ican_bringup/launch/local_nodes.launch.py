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
        # Camera node - Robust FFmpeg-based (uses working MediaMTX parameters)
        Node(
            package='ican_see',
            executable='cam_node_robust',
            name='camera_node',
            parameters=[{
                'device_id': 0,  # /dev/video0 (Insta360 X5)
                'width': 2880,  # Native resolution
                'height': 1440,
                'output_width': 1920,  # Scaled output
                'output_height': 960,
                'fps': 10,
                'publish_rate': 10.0,  # Publish at 10Hz
            }],
            remappings=[
                ('camera/image_raw', '/camera/image_raw'),
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
            executable='audio_playback_ffmpeg',
            name='audio_playback_ffmpeg',
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
