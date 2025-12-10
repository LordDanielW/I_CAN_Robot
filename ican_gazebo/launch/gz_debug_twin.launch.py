#!/usr/bin/env python3
"""
Gazebo Debug Digital Twin - Minimal Empty World with Real Robot Pose
Subscribes to pose estimates from real robot and displays in Gazebo.
No simulation - just visualization of real robot state.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for debug digital twin"""
    
    # Package paths
    go2_sdk_dir = get_package_share_directory('go2_robot_sdk')
    
    # Config paths
    urdf_path = os.path.join(go2_sdk_dir, 'urdf', 'go2.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    print(f"🐛 Debug Digital Twin Configuration:")
    print(f"   Subscribes to real robot pose estimates")
    print(f"   Empty Gazebo world for visualization")
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (false for real robot sync)'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', urdf_path]),
                    value_type=str
                ),
                'use_sim_time': use_sim_time
            }],
        ),
        
        # Launch Ignition Gazebo with empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 
                           'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                'gz_args': ['-r empty.sdf'],  # Empty world
            }.items(),
        ),
        
        # Spawn robot in Gazebo at origin
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_go2',
            output='screen',
            arguments=[
                '-name', 'go2',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.5',
            ],
        ),
        
        # Bridge real robot odometry to Gazebo for pose updates
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_pose',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                # Real robot odometry to Gazebo pose
                '/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry',
                
                # Gazebo clock (optional)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[
                ('/odom', '/go2_driver_node/odom'),  # From real robot
            ],
        ),
        
        # # Joint State Publisher for visualization
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     parameters=[{'use_sim_time': use_sim_time}],
        # ),
        
        # TF Static Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
