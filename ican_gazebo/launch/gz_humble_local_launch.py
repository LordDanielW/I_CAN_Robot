#!/usr/bin/env python3
"""
Gazebo Humble Local Launch - Robot Communication Only
Runs only nodes that communicate with the physical robot.
No debug visualizers (RViz, Foxglove, etc.)

Based on go2_ros2_sdk robot.launch.py
"""

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


class RobotConfig:
    """Configuration for robot connection"""
    
    def __init__(self):
        # Environment variables
        self.robot_token = os.getenv('ROBOT_TOKEN', '')
        self.robot_ip = os.getenv('ROBOT_IP', '')
        self.conn_type = os.getenv('CONN_TYPE', 'webrtc')
        
        # Package paths
        self.go2_sdk_dir = get_package_share_directory('go2_robot_sdk')
        
        # Config paths
        self.urdf_path = os.path.join(self.go2_sdk_dir, 'urdf', 'go2.urdf')
        self.twist_mux_config = os.path.join(self.go2_sdk_dir, 'config', 'twist_mux.yaml')
        self.joystick_config = os.path.join(self.go2_sdk_dir, 'config', 'joystick.yaml')
        
        print(f"🤖 Robot Local Configuration:")
        print(f"   Robot IP: {self.robot_ip}")
        print(f"   Connection: {self.conn_type}")
    
    def load_urdf(self) -> str:
        """Load URDF file content"""
        with open(self.urdf_path, 'r') as file:
            return file.read()


def generate_launch_description():
    """Generate launch description for local robot communication"""
    
    config = RobotConfig()
    
    # Launch arguments
    robot_ip = LaunchConfiguration('robot_ip', default=config.robot_ip)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_ip',
            default_value=config.robot_ip,
            description='IP address of the robot'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'enable_joystick',
            default_value='true',
            description='Enable joystick control'
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
        
        # Go2 Driver Node - Core robot communication
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'token': config.robot_token,
                'conn_type': config.conn_type,
                'enable_video': True,
                'decode_lidar': True,
                'publish_raw_voxel': True,
                'obstacle_avoidance': False,
            }],
            remappings=[
                ('cmd_vel_out', 'cmd_vel'),
            ],
        ),
        
        # Joystick Node
        Node(
            package='joy',
            executable='joy_node',
            parameters=[config.joystick_config]
        ),
        
        # Teleop Twist Joy
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='go2_teleop_node',
            parameters=[config.twist_mux_config],
        ),
        
        # Twist Mux - Manages multiple velocity command sources
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                config.twist_mux_config
            ],
        ),
        
        # Lidar to Pointcloud Node
        Node(
            package='lidar_processor_cpp',
            executable='lidar_to_pointcloud_node',
            name='lidar_to_pointcloud',
            parameters=[{
                'robot_ip_lst': [config.robot_ip],
                'map_name': '3d_map',
                'map_save': 'true'
            }],
        ),
        
        # Pointcloud Aggregator
        Node(
            package='lidar_processor_cpp',
            executable='pointcloud_aggregator_node',
            name='pointcloud_aggregator',
            parameters=[{
                'max_range': 20.0,
                'min_range': 0.1,
                'height_filter_min': -2.0,
                'height_filter_max': 3.0,
                'downsample_rate': 5,
                'publish_rate': 10.0
            }],
        ),
        
        # Pointcloud to Laserscan Converter
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 1.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', 'aggregated_pointcloud'),
                ('scan', 'scan'),
            ],
        ),
        
        # Image Compression for efficient transmission
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', 'camera/image_raw'),
                ('out/compressed', 'camera/compressed'),
            ],
        ),
    ])
