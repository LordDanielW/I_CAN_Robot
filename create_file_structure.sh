#!/bin/bash

# ##################################################
# I_CAN_Robot ROS 2 Architecture Generator
# Target: Ubuntu 24.04 (Jazzy) / Python Nodes
# ##################################################

# Define Workspace Paths
WS_PATH="$HOME/ros2_ws/src/I_CAN_Robot"

# Check if directory exists
if [ ! -d "$WS_PATH" ]; then
    echo "Target directory $WS_PATH does not exist."
    echo "Creating it now..."
    mkdir -p "$WS_PATH"
fi

cd "$WS_PATH"
echo "Generating ROS 2 Architecture in: $(pwd)"

# Helper function to create a python package with standard layout
create_pkg() {
    PKG_NAME=$1
    NODE_NAME=$2
    DESC=$3
    
    if [ -d "$PKG_NAME" ]; then
        echo "Package $PKG_NAME already exists. Skipping creation..."
    else
        echo "Creating package: $PKG_NAME..."
        ros2 pkg create --build-type ament_python --license Apache-2.0 --description "$DESC" "$PKG_NAME"
        
        # Create standard folders
        mkdir -p "$PKG_NAME/launch"
        mkdir -p "$PKG_NAME/config"
        mkdir -p "$PKG_NAME/resource"
    fi
}

# Helper function to inject a smart setup.py that handles launch/config files automatically
update_setup_py() {
    PKG_NAME=$1
    ENTRY_POINTS=$2 # String of entry points
    
    cd "$PKG_NAME"
    
    cat << EOF > setup.py
import os
from glob import glob
from setuptools import find_packages, setup

package_name = '$PKG_NAME'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Include URDF/Worlds if they exist (recursive)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fire',
    maintainer_email='user@todo.todo',
    description='Part of I_CAN_Robot stack',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            $ENTRY_POINTS
        ],
    },
)
EOF
    cd ..
}

# ##################################################
# 1. ican_description
# Holds URDF, Xacro, Meshes
# ##################################################
create_pkg "ican_description" "none" "Robot Description and URDFs"
mkdir -p ican_description/urdf
mkdir -p ican_description/meshes
# Update setup.py (No entry points needed for pure description pkg)
update_setup_py "ican_description" ""

# ##################################################
# 2. ican_gazebo
# Holds Worlds and Simulation Configs
# ##################################################
create_pkg "ican_gazebo" "none" "Gazebo Simulation Worlds and Plugins"
mkdir -p ican_gazebo/worlds
# Create a dummy world
echo "<?xml version='1.0'?> <sdf version='1.7'> <world name='default'> <include><uri>model://sun</uri></include> <include><uri>model://ground_plane</uri></include> </world> </sdf>" > ican_gazebo/worlds/empty.world
update_setup_py "ican_gazebo" ""

# ##################################################
# 3. ican_voice (The Ears)
# Whisper STT Node
# ##################################################
create_pkg "ican_voice" "voice_node" "Whisper Speech to Text Node"

# Write Node Stub
cat << EOF > ican_voice/ican_voice/voice_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# import faster_whisper # Uncomment when venv is active

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.publisher_ = self.create_publisher(String, '/human/speech', 10)
        self.get_logger().info('Ears are open (Whisper Node Initialized)')

    def listener_callback(self):
        # Logic for Faster-Whisper goes here
        msg = String()
        msg.data = "Hello Robot" 
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# Update Setup.py
update_setup_py "ican_voice" "'voice_node = ican_voice.voice_node:main',"

# ##################################################
# 4. ican_orchestrator (The Brain)
# Qwen 2.5 + MCP Host
# ##################################################
create_pkg "ican_orchestrator" "brain_node" "MCP Host and Qwen Logic"

cat << EOF > ican_orchestrator/ican_orchestrator/brain_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from mcp import ClientSession # Uncomment when venv is active

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self.subscription = self.create_subscription(
            String, '/human/speech', self.process_speech, 10)
        self.get_logger().info('Brain is active (Qwen 2.5 Orchestrator)')

    def process_speech(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
        # 1. Connect to MCP Servers
        # 2. Send context to Ollama/Qwen
        # 3. Execute MCP Tool
        pass

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

update_setup_py "ican_orchestrator" "'brain_node = ican_orchestrator.brain_node:main',"

# ##################################################
# 5. ican_mcp_server (The Hands & Senses)
# Exposes ROS 2 actions/topics as MCP Tools
# ##################################################
create_pkg "ican_mcp_server" "server" "MCP Server bridging ROS and LLM"

# Stub for Behavior Server (Actions)
cat << EOF > ican_mcp_server/ican_mcp_server/behavior_server.py
import rclpy
from rclpy.node import Node

class BehaviorServer(Node):
    def __init__(self):
        super().__init__('behavior_mcp_server')
        self.get_logger().info('Behavior Server Ready (Exposing Walk, Sit, Navigate)')
        # This is where you initialize the FastMCP server
        # and define tools that call ROS 2 Actions

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# Stub for Senses Server (Read Only)
cat << EOF > ican_mcp_server/ican_mcp_server/senses_server.py
import rclpy
from rclpy.node import Node

class SensesServer(Node):
    def __init__(self):
        super().__init__('senses_mcp_server')
        self.get_logger().info('Senses Server Ready (Exposing Battery, Lidar, Camera)')

def main(args=None):
    rclpy.init(args=args)
    node = SensesServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# Update setup.py with TWO entry points
update_setup_py "ican_mcp_server" "'behavior_server = ican_mcp_server.behavior_server:main',\n            'senses_server = ican_mcp_server.senses_server:main',"

# ##################################################
# 6. ican_bringup
# The Master Launch Files
# ##################################################
create_pkg "ican_bringup" "none" "Master Launch Files"

# Create Sim Launch
cat << EOF > ican_bringup/launch/sim_dog.launch.py
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
EOF

update_setup_py "ican_bringup" ""

# ##################################################
# Finalize
# ##################################################
echo "------------------------------------------------"
echo "Architecture Created Successfully at $WS_PATH"
echo "------------------------------------------------"
echo "Packages Created:"
echo "1. ican_description  (URDFs)"
echo "2. ican_gazebo       (Worlds)"
echo "3. ican_voice        (Whisper STT)"
echo "4. ican_orchestrator (Qwen Brain)"
echo "5. ican_mcp_server   (Tools for Brain)"
echo "6. ican_bringup      (Launch files)"
echo "------------------------------------------------"
echo "NEXT STEPS:"
echo "1. cd ~/ros2_ws"
echo "2. colcon build --symlink-install"
echo "3. source install/setup.bash"
echo "4. ros2 launch ican_bringup sim_dog.launch.py"