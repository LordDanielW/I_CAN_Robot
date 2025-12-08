#!/bin/bash
# #################################
# I_CAN Robot - Jazzy LOCAL Install
# Target: Ubuntu 24.04 + ROS 2 Jazzy
# Use Case: On-robot development, BLE audio I/O
# Includes: ROS 2 Jazzy, Gazebo Harmonic, dev tools, audio streaming
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
echo "I_CAN Robot - Jazzy LOCAL Install"
echo "User: $REAL_USER"
echo "Target: Ubuntu 24.04 + ROS 2 Jazzy"
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
# ROS 2 Jazzy
# #################################

echo "📦 Installing ROS 2 Jazzy..."

# Locale
apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 2 repo
apt install -y software-properties-common
add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update && apt upgrade -y
apt install -y ros-jazzy-desktop ros-dev-tools

# rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
sudo -u $REAL_USER rosdep update

# #################################
# Gazebo Harmonic
# #################################

echo "📦 Installing Gazebo Harmonic..."

apt install -y lsb-release gnupg
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

apt update
apt install -y gz-harmonic

# ROS-Gazebo bridge
apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image || true

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
alias ros_source='source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash 2>/dev/null'
alias dev_activate='source ~/ros2_dev_env/bin/activate'
alias go2_dog='ros2 launch unitree_go2 unitree_go2_launch.py'
alias gz_sim='gz sim'

# Auto-source ROS 2
source /opt/ros/jazzy/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
EOF
fi

# #################################
# Summary
# #################################

echo ""
echo "=========================================="
echo "✅ JAZZY LOCAL INSTALL COMPLETE"
echo "=========================================="
echo ""
echo "Installed:"
echo "  • ROS 2 Jazzy Desktop"
echo "  • Gazebo Harmonic"
echo "  • Audio streaming (PyAudio, PortAudio)"
echo "  • Dev tools (SSH, gdb, can-utils)"
echo ""
echo "Aliases:"
echo "  ros_source   - Source ROS 2 workspace"
echo "  dev_activate - Activate Python venv"
echo "  go2_dog      - Launch robot"
echo "  gz_sim       - Launch Gazebo"
echo ""
echo "⚡ This is LIGHTWEIGHT install for robot hardware."
echo "   For AI models, use: install_jazzy_remote.sh"
echo ""
echo "Restart terminal or: source ~/.bashrc"
echo "=========================================="