#!/bin/bash
# #################################
# I_CAN Robot - Humble LOCAL Install
# Target: Ubuntu 22.04 + ROS 2 Humble
# Use Case: On-robot development, BLE audio I/O
# Includes: ROS 2 Humble, Gazebo Ignition, dev tools, audio streaming
# Excludes: AI models (Ollama, Whisper, heavy TTS)
# #################################

set -e

# Check sudo
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: Run with sudo"
    echo "Usage: sudo bash $0"
    exit 1
fi

REAL_USER=${SUDO_USER:-$(whoami)}
USER_HOME="/home/$REAL_USER"

echo "=========================================="
echo "I_CAN Robot - Humble LOCAL Install"
echo "User: $REAL_USER"
echo "Target: Ubuntu 22.04 + ROS 2 Humble"
echo "=========================================="

# #################################
# Basic Tools
# #################################

echo "📦 Installing basic tools..."
apt update
apt install -y \
    git curl wget \
    build-essential \
    python3-pip python3-venv \
    htop tmux screen \
    net-tools iputils-ping \
    vim nano

# #################################
# ROS 2 Humble
# #################################

echo "📦 Installing ROS 2 Humble..."

# Locale
apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 2 repo
apt install -y software-properties-common
add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update && apt upgrade -y
apt install -y ros-humble-desktop ros-dev-tools

# rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
sudo -u $REAL_USER rosdep update

# #################################
# Gazebo Ignition Fortress
# #################################

echo "📦 Installing Gazebo Ignition Fortress..."

curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

apt update
apt install -y ignition-fortress

apt install -y \
    ros-humble-ros-gz \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-image

# #################################
# Audio Streaming (Lightweight)
# #################################

echo "📦 Installing audio streaming dependencies..."

apt install -y \
    portaudio19-dev \
    python3-pyaudio \
    libasound2-dev \
    alsa-utils \
    pulseaudio-utils

# #################################
# Python Dev Environment
# #################################

VENV_PATH="$USER_HOME/ros2_dev_env"

echo "📦 Creating Python virtual environment..."
sudo -u $REAL_USER python3 -m venv $VENV_PATH

sudo -u $REAL_USER $VENV_PATH/bin/pip install --upgrade pip
sudo -u $REAL_USER $VENV_PATH/bin/pip install \
    pytest ipython pyyaml numpy matplotlib pyaudio

# #################################
# Network & Debug Tools
# #################################

echo "📦 Installing debug utilities..."
apt install -y \
    openssh-server \
    can-utils i2c-tools usbutils \
    strace gdb

systemctl enable ssh
systemctl start ssh

# #################################
# Bash Aliases
# #################################

if ! grep -q "# I_CAN Robot Aliases" $USER_HOME/.bashrc; then
    cat >> $USER_HOME/.bashrc << 'EOF'

# I_CAN Robot Aliases
alias ros_source='source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash 2>/dev/null'
alias dev_activate='source ~/ros2_dev_env/bin/activate'
alias go2_dog='ros2 launch unitree_go2 unitree_go2_launch.py'
alias ign_gazebo='ign gazebo'

# Auto-source ROS 2
source /opt/ros/humble/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
EOF
fi

# #################################
# Summary
# #################################

echo ""
echo "=========================================="
echo "✅ HUMBLE LOCAL INSTALL COMPLETE"
echo "=========================================="
echo ""
echo "Installed:"
echo "  • ROS 2 Humble Desktop"
echo "  • Gazebo Ignition Fortress"
echo "  • Audio streaming (PyAudio, PortAudio)"
echo "  • Dev tools (SSH, gdb, can-utils)"
echo ""
echo "Aliases:"
echo "  ros_source   - Source ROS 2 workspace"
echo "  dev_activate - Activate Python venv"
echo "  go2_dog      - Launch robot"
echo "  ign_gazebo   - Launch Gazebo"
echo ""
echo "⚡ This is LIGHTWEIGHT install for robot hardware."
echo "   For AI models, use: install_humble_remote.sh"
echo ""
echo "Restart terminal or: source ~/.bashrc"
echo "=========================================="