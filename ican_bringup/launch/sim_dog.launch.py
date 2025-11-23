from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    unitree_sim_path = get_package_share_directory('unitree_go2_sim')

    return LaunchDescription([
        # 1. Launch the Unitree Go2 Simulation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(unitree_sim_path, 'launch', 'unitree_go2_launch.py')
            )
        ),
        
        # 2. Launch Our Brain
        Node(
            package='ican_orchestrator',
            executable='brain_node',
            name='brain_node',
            output='screen'
        ),

        # 3. Launch Our Ears
        Node(
            package='ican_voice',
            executable='voice_node',
            name='voice_node',
            output='screen'
        ),
        
        # 4. Launch MCP Bridge
        Node(
            package='ican_mcp_server',
            executable='behavior_server',
            name='behavior_server',
            output='screen'
        ),
    ])
