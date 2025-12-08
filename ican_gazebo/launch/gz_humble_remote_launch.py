#!/usr/bin/env python3
"""
Gazebo Humble Remote Launch - Digital Twin with Visualizers
Runs digital twin in Gazebo (empty world) with RViz, Foxglove, and rqt_graph.
Does NOT connect to physical robot.

Based on go2_ros2_sdk robot.launch.py
"""

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


class RemoteConfig:
    """Configuration for remote visualization and simulation"""
    
    def __init__(self):
        # Package paths
        self.go2_sdk_dir = get_package_share_directory('go2_robot_sdk')
        self.ican_gazebo_dir = get_package_share_directory('ican_gazebo')
        
        # Config paths
        self.urdf_path = os.path.join(self.go2_sdk_dir, 'urdf', 'go2.urdf')
        self.rviz_config = os.path.join(self.go2_sdk_dir, 'config', 'single_go2.rviz')
        self.world_path = os.path.join(self.ican_gazebo_dir, 'worlds', 'empty.world')
        
        print(f"🌐 Remote Digital Twin Configuration:")
        print(f"   World: {self.world_path}")
        print(f"   RViz Config: {self.rviz_config}")
    
    def load_urdf(self) -> str:
        """Load URDF file content"""
        with open(self.urdf_path, 'r') as file:
            return file.read()


def generate_launch_description():
    """Generate launch description for remote digital twin"""
    
    config = RemoteConfig()
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_init_x = LaunchConfiguration('world_init_x', default='0.0')
    world_init_y = LaunchConfiguration('world_init_y', default='0.0')
    world_init_z = LaunchConfiguration('world_init_z', default='0.5')
    world_init_heading = LaunchConfiguration('world_init_heading', default='0.0')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world_init_x',
            default_value='0.0',
            description='Initial X position in world'
        ),
        DeclareLaunchArgument(
            'world_init_y',
            default_value='0.0',
            description='Initial Y position in world'
        ),
        DeclareLaunchArgument(
            'world_init_z',
            default_value='0.5',
            description='Initial Z position in world'
        ),
        DeclareLaunchArgument(
            'world_init_heading',
            default_value='0.0',
            description='Initial heading (yaw) in world'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', config.urdf_path]),
                    value_type=str
                ),
                'use_sim_time': use_sim_time
            }],
        ),
        
        # Launch Gazebo with empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 
                           'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={
                'world': config.world_path,
                'verbose': 'true',
                'pause': 'false',
            }.items(),
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_go2',
            output='screen',
            arguments=[
                '-entity', 'go2',
                '-topic', 'robot_description',
                '-x', world_init_x,
                '-y', world_init_y,
                '-z', world_init_z,
                '-Y', world_init_heading,
            ],
        ),
        
        # Joint State Publisher (for simulation)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # RViz2 - Main visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config.rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # Foxglove Bridge - Web-based visualization
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'send_buffer_limit': 100000000,
                'use_sim_time': use_sim_time,
            }],
        ),
        
        # rqt_graph - Node/topic visualization
        ExecuteProcess(
            cmd=['rqt_graph'],
            output='screen',
            name='rqt_graph',
        ),
        
        # TF Static Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # Simulated sensors (placeholder for camera/lidar)
        Node(
            package='image_transport',
            executable='republish',
            name='sim_camera_republisher',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', 'camera/image_raw'),
                ('out/compressed', 'camera/compressed'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
