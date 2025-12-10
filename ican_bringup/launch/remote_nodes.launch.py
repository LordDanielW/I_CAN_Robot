#!/usr/bin/env python3
"""
Remote Nodes Launch File
Runs all voice-controlled robot nodes with tool calling capability.
Launch this on a powerful remote computer for best performance.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    ican_brain_dir = get_package_share_directory('ican_brain')
    ican_tools_dir = get_package_share_directory('ican_tools')
    ican_voice_dir = get_package_share_directory('ican_voice')
    go2_config_dir = get_package_share_directory('go2_config')
    
    return LaunchDescription([
        # # Include Gazebo Simulation
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(go2_config_dir, 'launch', 'gazebo.launch.py')
        #     ),
        #     launch_arguments={'rviz': 'true'}.items()
        # ),

        # Just run with this seperately
        # ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
        # ollama run qwen3-vl:8b
        
        # Include Brain Remote Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ican_brain_dir, 'launch', 'ican_brain_remote.launch.py')
            )
        ),
        
        # Include Tools Remote Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ican_tools_dir, 'launch', 'ican_tools_remote.launch.py')
            )
        ),
        
        # Include Voice Remote Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ican_voice_dir, 'launch', 'ican_voice_remote.launch.py')
            )
        ),
    ])

