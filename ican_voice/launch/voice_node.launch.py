#!/usr/bin/env python3
"""
Launch file for the Voice Node (The Ears)

This launch file starts the voice_node with configurable parameters.
You can override parameters from the command line:
    ros2 launch ican_voice voice_node.launch.py model_size:=tiny.en
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for voice node."""
    
    # Declare launch arguments
    model_size_arg = DeclareLaunchArgument(
        'model_size',
        default_value='base.en',
        description='Whisper model size: tiny.en, base.en, small.en, medium.en, large-v2, etc.'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='auto',
        description='Device to run Whisper on: auto, cuda, or cpu'
    )
    
    compute_type_arg = DeclareLaunchArgument(
        'compute_type',
        default_value='int8',
        description='Compute type: int8, float16, or float32'
    )
    
    mic_device_index_arg = DeclareLaunchArgument(
        'mic_device_index',
        default_value='-1',
        description='Microphone device index (-1 for default)'
    )
    
    energy_threshold_arg = DeclareLaunchArgument(
        'energy_threshold',
        default_value='500.0',
        description='RMS energy threshold for speech detection'
    )
    
    silence_duration_arg = DeclareLaunchArgument(
        'silence_duration',
        default_value='1.5',
        description='Seconds of silence to end a phrase'
    )
    
    # Create the voice node
    voice_node = Node(
        package='ican_voice',
        executable='voice_node',
        name='voice_node',
        output='screen',
        parameters=[{
            'model_size': LaunchConfiguration('model_size'),
            'device': LaunchConfiguration('device'),
            'compute_type': LaunchConfiguration('compute_type'),
            'mic_device_index': LaunchConfiguration('mic_device_index'),
            'energy_threshold': LaunchConfiguration('energy_threshold'),
            'silence_duration': LaunchConfiguration('silence_duration'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        model_size_arg,
        device_arg,
        compute_type_arg,
        mic_device_index_arg,
        energy_threshold_arg,
        silence_duration_arg,
        voice_node,
    ])
