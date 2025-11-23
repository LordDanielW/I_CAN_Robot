#!/bin/bash

# Define the root of the repo
REPO_ROOT="$HOME/ros2_ws/src/I_CAN_Robot"

echo "Fixing ROS 2 Package issues in $REPO_ROOT..."

# ==========================================
# FIX 1: Repair ican_mcp_server setup.py
# The syntax error was caused by bad newline formatting in the list
# ==========================================
echo "Repairing ican_mcp_server/setup.py..."
cd "$REPO_ROOT/ican_mcp_server"

cat << EOF > setup.py
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ican_mcp_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fire',
    maintainer_email='user@todo.todo',
    description='MCP Server bridging ROS and LLM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_server = ican_mcp_server.behavior_server:main',
            'senses_server = ican_mcp_server.senses_server:main',
        ],
    },
)
EOF

# ==========================================
# FIX 2: Create Missing Resource Marker Files
# This fixes "error: can't copy 'resource/ican_bringup': doesn't exist"
# ==========================================
echo "Ensuring resource marker files exist..."

# Function to touch resource files
fix_resource() {
    PKG=$1
    if [ -d "$REPO_ROOT/$PKG" ]; then
        mkdir -p "$REPO_ROOT/$PKG/resource"
        # The file MUST be named exactly the same as the package
        touch "$REPO_ROOT/$PKG/resource/$PKG"
        echo " - Fixed resource for $PKG"
    fi
}

fix_resource "ican_bringup"
fix_resource "ican_description"
fix_resource "ican_gazebo"
fix_resource "ican_mcp_server"
fix_resource "ican_orchestrator"
fix_resource "ican_voice"

echo "=========================================="
echo "Repairs Complete."
echo "Please run: cd ~/ros2_ws && colcon build --symlink-install"
echo "=========================================="