#!/usr/bin/env python3
"""
Debug Launch File
Launches debugging and visualization tools for the I_CAN Robot system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription([
        # RQT Graph - Visualize the node graph
        ExecuteProcess(
            cmd=['rqt_graph'],
            output='screen',
            name='rqt_graph'
        ),
        
        # RViz - 3D Visualization (check if already running via condition)
        ExecuteProcess(
            cmd=['bash', '-c', 'pgrep -f rviz2 > /dev/null || rviz2'],
            output='screen',
            shell=True,
            name='rviz2'
        ),
        
        # Debug Monitor - Monitor system state and topics
        Node(
            package='ican_brain',
            executable='debug_monitor',
            name='debug_monitor',
            output='screen'
        ),
    ])
