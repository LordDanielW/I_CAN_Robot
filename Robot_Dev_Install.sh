#!/bin/bash
# #################################
# I_CAN_Robot Lightweight Dev Installation
# For on-robot development and debugging
# Excludes: Ollama, Qwen, Gazebo (heavy components)
# Includes: ROS 2 Jazzy, dev tools, debugging utilities
# Target: Ubuntu 24.04 on physical robot hardware
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

echo "=========================================="
echo "I_CAN_Robot Lightweight Dev Install"
echo "User: $REAL_USER at $USER_HOME"
echo "=========================================="

# #################################
# Basic Tools & Dependencies
# #################################

echo "Installing basic tools..."
sudo apt update
sudo apt install -y \
    git curl wget \
    build-essential \
    python3-pip python3-venv \
    htop tmux screen \
    net-tools iputils-ping \
    vim nano

# #################################
# ROS 2 Jazzy Install
# #################################

echo "Installing ROS 2 Jazzy..."

# Set locale
locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 Desktop
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
sudo -u $REAL_USER rosdep update

# #################################
# Python Development Environment
# #################################

VENV_PATH="$USER_HOME/ros2_dev_env"

echo "Creating Python Virtual Environment at $VENV_PATH"
sudo -u $REAL_USER python3 -m venv $VENV_PATH

# Install useful Python packages for debugging
echo "Installing Python development dependencies..."
sudo -u $REAL_USER $VENV_PATH/bin/pip install --upgrade pip
sudo -u $REAL_USER $VENV_PATH/bin/pip install \
    pytest \
    ipython \
    pyyaml \
    numpy \
    matplotlib

# #################################
# Network & Debugging Tools
# #################################

echo "Installing network and debugging utilities..."
sudo apt install -y \
    openssh-server \
    can-utils \
    i2c-tools \
    usbutils \
    strace \
    gdb

# Enable SSH service
sudo systemctl enable ssh
sudo systemctl start ssh

# #################################
# Bash Aliases Setup
# #################################

# Add convenient aliases to .bashrc (avoid duplicates)
if ! grep -q "# ROS 2 MCP & Unitree Aliases" $USER_HOME/.bashrc; then
    echo "" >> $USER_HOME/.bashrc
    echo "# ROS 2 MCP & Unitree Aliases" >> $USER_HOME/.bashrc
    echo "alias mcp_activate='source /opt/ros/jazzy/setup.bash && source ~/ros2_mcp_env/bin/activate'" >> $USER_HOME/.bashrc
    echo "alias ros_source='source /opt/ros/jazzy/setup.bash && source /home/$REAL_USER/ros2_ws/install/setup.bash'" >> $USER_HOME/.bashrc
    echo "alias go2_dog='ros2 launch unitree_go2_sim unitree_go2_launch.py'" >> $USER_HOME/.bashrc
    echo "" >> $USER_HOME/.bashrc
    echo "ros_source" >> $USER_HOME/.bashrc
fi

echo ""
echo "=========================================="
echo "DEV INSTALLATION COMPLETE"
echo "=========================================="
echo "Installed:"
echo "  - ROS 2 Jazzy (Desktop)"
echo "  - Python Dev Environment"
echo "  - Debugging Utilities (htop, gdb, can-utils)"
echo "  - SSH Server (enabled)"
echo ""
echo "Aliases added to .bashrc:"
echo "  mcp_activate  - Activate MCP environment"
echo "  ros_source    - Source ROS 2 & workspace"
echo "  go2_dog       - Launch on robot"
echo ""
echo "This is a lightweight install for on-robot use."
echo "For simulation, use Robot_Full_Install.sh"
echo "Restart terminal or run: source ~/.bashrc"
echo "=========================================="
