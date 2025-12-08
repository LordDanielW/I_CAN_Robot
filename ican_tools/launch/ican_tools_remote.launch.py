#!/usr/bin/env python3
"""
I_CAN Tools Remote Launch File
Launches the tool management and service nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Tool Manager - Orchestrates tool discovery and execution
        Node(
            package='ican_tools',
            executable='tool_manager_node',
            name='tool_manager_node',
            output='screen'
        ),
        
        # Dice Service - Example tool for RPG dice rolling
        Node(
            package='ican_tools',
            executable='dice_service_node',
            name='dice_service_node',
            output='screen'
        ),
        
        # Move Robot - Preprogrammed movement paths
        Node(
            package='ican_tools',
            executable='move_robot_node',
            name='move_robot_node',
            output='screen'
        ),
        
        # Query Room - Vision-based room description using VLM
        Node(
            package='ican_tools',
            executable='query_room_node',
            name='query_room_node',
            output='screen'
        ),
    ])
