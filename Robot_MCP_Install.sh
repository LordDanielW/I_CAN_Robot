#!/bin/bash
# #################################
# Basic Includes & Checks
# #################################

# Check if script is run as root/sudo
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: This script must be run with sudo"
    echo "Usage: sudo bash $0"
    exit 1
fi

# Identify the real user (who called sudo) to install files in their home dir
REAL_USER=${SUDO_USER:-$(whoami)}
USER_HOME="/home/$REAL_USER"

echo "Installing for user: $REAL_USER at $USER_HOME"

# Install basic tools
sudo apt update
sudo apt install -y git curl build-essential python3-pip python3-venv

# #################################
# ROS 2 Jazzy Install
# From https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# #################################

# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 Desktop (includes Rviz, demos)
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ament-cmake
# Install common robot state publishers often needed for dogs/robots
sudo apt install -y ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-xacro

# #################################
# Unitree Go2 & Gazebo Dependencies
# Dependencies to talk to the dog and simulate it
# #################################

echo "Installing Unitree Go2 and Gazebo Sim dependencies..."

# ROS 2 Control & Gazebo Bridge
sudo apt install -y ros-jazzy-gazebo-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-robot-localization

# Lidar/Sensor dependencies for the Go2
sudo apt install -y ros-jazzy-velodyne ros-jazzy-velodyne-description

# #################################
# Ollama & Qwen 2.5 Install
# The "Brain" of the architecture
# #################################

echo "Installing Ollama..."
curl -fsSL https://ollama.com/install.sh | sh

# Wait for Ollama service to spin up
echo "Waiting for Ollama service..."
sleep 5

# Pull Qwen 2.5 (7B is the standard efficient model for local robotics)
# Run as the real user so permissions are correct
echo "Pulling Qwen 2.5 model..."
sudo -u $REAL_USER ollama pull qwen2.5:7b

# #################################
# Audio & Voice Dependencies
# For Whisper (Ear) and PyAudio
# #################################

# Install system audio libraries required by PyAudio and Faster-Whisper
sudo apt install -y portaudio19-dev python3-pyaudio libasound2-dev ffmpeg

# #################################
# MCP & Python Environment Setup
# Ubuntu 24.04 enforces PEP 668, so we use a venv
# #################################

VENV_PATH="$USER_HOME/ros2_mcp_env"

echo "Creating Python Virtual Environment for MCP at $VENV_PATH"
sudo -u $REAL_USER python3 -m venv $VENV_PATH

# Install Python Libraries into the venv
# mcp: The Protocol SDK
# openai: Client to talk to Ollama
# faster-whisper: Optimized local speech-to-text
# pyaudio: Microphone access
# numpy: Data handling
echo "Installing Python dependencies (mcp, openai, whisper)..."
sudo -u $REAL_USER $VENV_PATH/bin/pip install mcp openai faster-whisper pyaudio numpy

# #################################
# ROS 2 Websocket & Bridge
# Required for Web UIs or non-native integrations
# #################################
sudo apt install -y ros-jazzy-rosbridge-suite ros-jazzy-rosapi

# #################################
# Gazebo Harmonic (Sim) Install
# #################################

sudo apt-get install -y lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic

# #################################
# Workspace Setup & Unitree Simulation
# #################################

# Create a workspace for the MCP node and Unitree Sim
WS_DIR="$USER_HOME/ros2_mcp_ws"
mkdir -p $WS_DIR/src

echo "Cloning Unitree Go2 ROS 2 stack (CHAMP + Gazebo Harmonic)..."
# Clone the Unitree Go2 repo recommended for Jazzy
cd $WS_DIR/src
# We clone as the real user to ensure file ownership is correct
sudo -u $REAL_USER git clone https://github.com/khaledgabr77/unitree_go2_ros2

# Install dependencies via rosdep
echo "Installing workspace dependencies via rosdep..."
cd $WS_DIR
# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
# Update rosdep as the user
sudo -u $REAL_USER rosdep update
# Install dependencies (needs sudo for apt installs)
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo "Building the workspace (this may take a moment)..."
# Fix permissions just in case before building
sudo chown -R $REAL_USER:$REAL_USER $WS_DIR
# Build as the user
sudo -u $REAL_USER colcon build --symlink-install

# #################################
# Aliases & Cleanup
# #################################

# Add convenient aliases to .bashrc
echo "" >> $USER_HOME/.bashrc
echo "# ROS 2 MCP & Unitree Aliases" >> $USER_HOME/.bashrc
echo "alias mcp_activate='source /opt/ros/jazzy/setup.bash && source ~/ros2_mcp_env/bin/activate'" >> $USER_HOME/.bashrc
echo "alias ros_source='source /opt/ros/jazzy/setup.bash && source $WS_DIR/install/setup.bash'" >> $USER_HOME/.bashrc
# Alias to quickly launch the simulation
echo "alias go2_sim='ros2 launch unitree_go2_sim unitree_go2_launch.py'" >> $USER_HOME/.bashrc

echo "#########################################################"
echo "INSTALLATION COMPLETE"
echo "1. To use the AI node, run: mcp_activate"
echo "2. To launch the Unitree Go2 Sim, run: ros_source && go2_sim"
echo "3. To run Qwen manually: ollama run qwen2.5:7b"
echo "#########################################################"