#!/bin/bash
# #################################
# I_CAN_Robot Full Installation
# Includes: ROS 2 Jazzy, Gazebo Harmonic, Ollama + Qwen 2.5
# Target: Ubuntu 24.04 Native or WSL
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
echo "I_CAN_Robot Full Installation"
echo "User: $REAL_USER at $USER_HOME"
echo "=========================================="

# #################################
# Basic Tools & Dependencies
# #################################

echo "Installing basic tools..."
sudo apt update
sudo apt install -y git curl wget build-essential python3-pip python3-venv

# #################################
# Chrome & VS Code Installation
# #################################

# Check and install Google Chrome
if ! command -v google-chrome &> /dev/null; then
    echo "Installing Google Chrome..."
    wget -q -O /tmp/google-chrome-stable_current_amd64.deb https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
    sudo apt install -y /tmp/google-chrome-stable_current_amd64.deb
    rm /tmp/google-chrome-stable_current_amd64.deb
else
    echo "Google Chrome already installed"
fi

# Check and install VS Code
if ! command -v code &> /dev/null; then
    echo "Installing VS Code..."
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /tmp/packages.microsoft.gpg
    sudo install -D -o root -g root -m 644 /tmp/packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
    sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
    rm -f /tmp/packages.microsoft.gpg
    sudo apt update
    sudo apt install -y code
else
    echo "VS Code already installed"
fi

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
# Gazebo Harmonic (Sim) Install
# #################################

echo "Installing Gazebo Harmonic..."
sudo apt-get install -y lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic

# #################################
# MCP & Python Environment Setup
# #################################

VENV_PATH="$USER_HOME/ros2_mcp_env"

echo "Creating Python Virtual Environment for MCP at $VENV_PATH"
sudo -u $REAL_USER python3 -m venv $VENV_PATH

# Install Python Libraries into the venv
echo "Installing Python dependencies (mcp, openai)..."
sudo -u $REAL_USER $VENV_PATH/bin/pip install --upgrade pip
sudo -u $REAL_USER $VENV_PATH/bin/pip install mcp openai


# #################################
# Whisper Install
# #################################

# (OpenAI Speech to Text)
echo "Installing Faster Whisper..."
sudo apt-get install -y portaudio19-dev python3-pyaudio
/usr/bin/python3 -m pip install --break-system-packages pyaudio faster-whisper

# #################################
# YOLOv13 Vision Install
# #################################

echo "Installing YOLO dependencies..."
/usr/bin/python3 -m pip install --break-system-packages ultralytics opencv-python torch torchvision

echo ""
echo "YOLOv8 (stable) is ready - models auto-download on first use"
echo ""
echo "Optional: Install YOLOv13 (latest, +3% accuracy)?"
read -p "Clone YOLOv13 repository? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if [ ! -d "$USER_HOME/yolov13" ]; then
        echo "Cloning YOLOv13 repository..."
        sudo -u $REAL_USER git clone https://github.com/iMoonLab/yolov13.git $USER_HOME/yolov13
        echo "YOLOv13 repository cloned to $USER_HOME/yolov13"
        echo "Note: Download pre-trained models from: https://github.com/iMoonLab/yolov13/releases"
    else
        echo "YOLOv13 repository already exists at $USER_HOME/yolov13"
    fi
else
    echo "Skipping YOLOv13. Using stable YOLOv8 (recommended)."
fi

# #################################
# Ollama & Qwen 2.5 Install
# #################################

echo "Installing Ollama..."
curl -fsSL https://ollama.com/install.sh | sh

# Wait for Ollama service to spin up
echo "Waiting for Ollama service..."
sleep 5

# Pull Qwen 2.5
echo "Pulling Qwen 2.5 model (this may take a while)..."
sudo -u $REAL_USER ollama pull qwen2.5:7b

# #################################
# Bash Aliases Setup
# #################################

# Add convenient aliases to .bashrc (avoid duplicates)
if ! grep -q "# ROS 2 MCP & Unitree Aliases" $USER_HOME/.bashrc; then
    echo "" >> $USER_HOME/.bashrc
    echo "# ROS 2 MCP & Unitree Aliases" >> $USER_HOME/.bashrc
    echo "alias mcp_activate='source /opt/ros/jazzy/setup.bash && source ~/ros2_mcp_env/bin/activate'" >> $USER_HOME/.bashrc
    echo "alias ros_source='source /opt/ros/jazzy/setup.bash && source /home/$REAL_USER/ros2_ws/install/setup.bash'" >> $USER_HOME/.bashrc
    echo "alias go2_sim='ros2 launch unitree_go2_sim unitree_go2_launch.py'" >> $USER_HOME/.bashrc
    echo "" >> $USER_HOME/.bashrc
    echo "ros_source" >> $USER_HOME/.bashrc
fi

echo ""
echo "=========================================="
echo "FULL INSTALLATION COMPLETE"
echo "=========================================="
echo "Installed:"
echo "  - ROS 2 Jazzy"
echo "  - Gazebo Harmonic"
echo "  - Ollama + Qwen 2.5"
echo "  - Faster Whisper (Speech Recognition)"
echo "  - YOLOv8 (Object Detection, stable)"
echo "  - Chrome & VS Code"
echo "  - MCP Python Environment"
echo ""
echo "Aliases added to .bashrc:"
echo "  mcp_activate  - Activate MCP environment"
echo "  ros_source    - Source ROS 2 & workspace"
echo "  go2_sim       - Launch simulation"
echo ""
echo "Run 'ollama run qwen2.5:7b' to test AI"
echo "Restart terminal or run: source ~/.bashrc"
echo "=========================================="
