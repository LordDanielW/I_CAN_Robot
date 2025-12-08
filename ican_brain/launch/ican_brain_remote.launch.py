#!/usr/bin/env python3
"""
I_CAN Brain Remote Launch File
Launches the AI brain nodes for the voice-controlled robot system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Prompt Node - Wake word detection & context building
        Node(
            package='ican_brain',
            executable='prompt_node',
            name='prompt_node',
            output='screen',
            parameters=[{
                'wake_word': 'hey spot',
                'buffer_timeout': 2.0,
                'wake_word_threshold': 0.75
            }]
        ),
        
        # Ollama Tool Node - AI brain for natural language understanding with tool calling
        Node(
            package='ican_brain',
            executable='ollama_node',
            name='ollama_node',
            output='screen',
            parameters=[{
                'llm_model': 'qwen2.5:7b'
            }]
        ),
    ])
