#!/usr/bin/env python3
"""
Gazebo Humble Remote Launch - Full Simulation with Indoor Environment
Runs complete simulation of Unitree Go2 in indoor building/room environment.
Simulates all joints, physics, and responds to cmd_vel commands.
Does NOT connect to physical robot - pure simulation.

Uses Ignition Gazebo (ros_gz_sim) compatible with ROS 2 Humble.
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
from launch.conditions import IfCondition
from ament_index_python.packages import PackageNotFoundError


class RemoteConfig:
    """Configuration for remote visualization and simulation"""
    
    def __init__(self):
        # Package paths
        self.go2_sdk_dir = get_package_share_directory('go2_robot_sdk')
        self.ican_gazebo_dir = get_package_share_directory('ican_gazebo')
        
        # Config paths
        self.urdf_path = os.path.join(self.go2_sdk_dir, 'urdf', 'go2.urdf')
        self.rviz_config = os.path.join(self.go2_sdk_dir, 'config', 'single_go2.rviz')
        
        # Check for optional packages
        self.has_foxglove = self._check_package('foxglove_bridge')
        
        print(f"🏢 Full Simulation Configuration:")
        print(f"   World: Indoor building environment")
        print(f"   Physics: Full joint simulation + cmd_vel control")
        print(f"   RViz Config: {self.rviz_config}")
        if not self.has_foxglove:
            print(f"   ⚠️  Foxglove Bridge not available (optional)")
    
    def _check_package(self, package_name: str) -> bool:
        """Check if a package is available"""
        try:
            get_package_share_directory(package_name)
            return True
        except PackageNotFoundError:
            return False
    
    def load_urdf(self) -> str:
        """Load URDF file content"""
        with open(self.urdf_path, 'r') as file:
            return file.read()


def generate_launch_description():
    """Generate launch description for remote digital twin"""
    
    config = RemoteConfig()
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_foxglove = LaunchConfiguration('enable_foxglove', default=str(config.has_foxglove).lower())
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
            'enable_foxglove',
            default_value=str(config.has_foxglove).lower(),
            description='Enable Foxglove Bridge (if available)'
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
        
        # Launch Ignition Gazebo with empty world
        # Launch Ignition Gazebo with indoor building world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 
                           'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                # Use default_world.sdf or office.sdf for indoor environment
                # Available worlds: default_world.sdf, office.sdf, shapes.sdf
                'gz_args': ['-r -v 4 /usr/share/gz/gz-sim8/worlds/default.sdf'],
            }.items(),
        ),
        # Spawn robot in Ignition Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_go2',
            output='screen',
            arguments=[
                '-name', 'go2',
                '-topic', 'robot_description',
                '-x', world_init_x,
                '-y', world_init_y,
                '-z', world_init_z,
                '-Y', world_init_heading,
            ],
        # Bridge ROS 2 topics to Ignition Gazebo for full simulation
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                # Gazebo to ROS - simulation outputs
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/go2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/model/go2/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
                '/model/go2/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                
                # ROS to Gazebo - control inputs
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            remappings=[
                ('/model/go2/odometry', '/odom'),
                ('/model/go2/joint_state', '/joint_states'),
            ],
        ),
        
        # Simulated Robot Controller - converts cmd_vel to joint commands
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),      '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
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
        
        # Foxglove Bridge - Web-based visualization (optional)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        # TF Static Transform (map frame)
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
        
        # Simulated Camera (if available in model)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_camera',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            ],
        ),
        
        # Simulated LiDAR (if available in model)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_lidar',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ],
        ),
    ])
