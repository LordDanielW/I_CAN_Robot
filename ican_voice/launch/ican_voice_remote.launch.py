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
                'model_size': 'large-v3-turbo',
                'device': 'cuda',
                'compute_type': 'float16',
                'batch_size': 16,
                'beam_size': 1
            }]
        ),
        
        # Kokoro TTS - Text-to-speech synthesis
        Node(
            package='ican_voice',
            executable='tts_node',
            name='tts_streamer',
            output='screen',
            parameters=[{
                'voice': 'af_heart',
                'lang_code': 'a',
                'speed': 1.0,
                'device': 'cuda',
                'use_kokoro': True
            }]
        ),
    ])
