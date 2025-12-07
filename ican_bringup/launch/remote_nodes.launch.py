#!/usr/bin/env python3
"""
Remote Nodes Launch File
Runs all voice-controlled robot nodes with tool calling capability.
Launch this on a powerful remote computer for best performance.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Speech Recognition - Converts audio to text
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
        
        # Prompt Node - Wake word detection & context building
        Node(
            package='ican_orchestrator',
            executable='prompt_node',
            name='prompt_node',
            output='screen',
            parameters=[{
                'wake_word': 'hey spot',
                'buffer_timeout': 2.0,
                'wake_word_threshold': 0.75
            }]
        ),
        
        # Ollama LLM - AI brain for natural language understanding
        Node(
            package='ican_orchestrator',
            executable='ollama_tool_node',
            name='ollama_tool_node',
            output='screen',
            parameters=[{
                'llm_model': 'qwen2.5:7b'
            }]
        ),
        
        # Tool Manager - Orchestrates tool discovery and execution
        Node(
            package='ican_mcp_server',
            executable='tool_manager_node',
            name='tool_manager_node',
            output='screen'
        ),
        
        # Dice Service - Example tool for RPG dice rolling
        Node(
            package='ican_mcp_server',
            executable='dice_service_node',
            name='dice_service_node',
            output='screen'
        ),
    ])

