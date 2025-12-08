#!/bin/bash
# #################################
# I_CAN Robot - Humble REMOTE Install
# Target: Ubuntu 22.04 + ROS 2 Humble + RTX GPU
# Use Case: Workstation with GPU for AI processing
# Includes: ROS 2 Humble, Gazebo, Ollama, Whisper, Kokoro TTS, YOLO
# Optimized for: RTX 5070 Ti / RTX 4090 / RTX 3090
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
echo "I_CAN Robot - Humble REMOTE Install"
echo "User: $REAL_USER"
echo "Target: Ubuntu 22.04 + ROS 2 Humble + GPU AI"
echo "=========================================="

# #################################
# Basic Tools
# #################################

echo "📦 Installing basic tools..."
apt update
apt install -y \
    git curl wget \
    build-essential cmake \
    python3-pip python3-venv python3-dev \
    htop tmux screen \
    net-tools \
    vim nano \
    espeak-ng  # Required for Kokoro TTS

# #################################
# Chrome & VS Code (Optional)
# #################################

if ! command -v google-chrome &> /dev/null; then
    echo "📦 Installing Google Chrome..."
    wget -q -O /tmp/chrome.deb https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
    apt install -y /tmp/chrome.deb
    rm /tmp/chrome.deb
fi

if ! command -v code &> /dev/null; then
    echo "📦 Installing VS Code..."
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /tmp/ms.gpg
    install -D -o root -g root -m 644 /tmp/ms.gpg /etc/apt/keyrings/packages.microsoft.gpg
    echo "deb [arch=amd64,arm64 signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list
    rm /tmp/ms.gpg
    apt update && apt install -y code
fi

# #################################
# ROS 2 Humble
# #################################

echo "📦 Installing ROS 2 Humble..."

apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

apt install -y software-properties-common
add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update && apt upgrade -y
apt install -y ros-humble-desktop ros-dev-tools

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
# Audio Dependencies
# #################################

echo "📦 Installing audio dependencies..."
apt install -y \
    portaudio19-dev \
    python3-pyaudio \
    libasound2-dev \
    alsa-utils \
    pulseaudio-utils \
    ffmpeg

# #################################
# Python AI Packages (System Install)
# #################################
# NOTE: We install to SYSTEM Python, not venv, because ROS2's colcon
# generates entry points with #!/usr/bin/python3 hardcoded.
# Using --break-system-packages is safe - it just bypasses PEP 668.

echo "📦 Installing Python packages to SYSTEM Python..."
echo "   (Required for ros2 run compatibility)"

# Upgrade pip first
/usr/bin/python3 -m pip install --break-system-packages --upgrade pip setuptools wheel

# Core packages
/usr/bin/python3 -m pip install --break-system-packages \
    mcp openai pyyaml

# PyTorch with CUDA 12.1 (for RTX 50/40/30 series)
echo "📦 Installing PyTorch with CUDA 12.1..."
/usr/bin/python3 -m pip install --break-system-packages \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# #################################
# Faster-Whisper (Speech Recognition)
# #################################

echo "📦 Installing Faster-Whisper (large-v3-turbo)..."
/usr/bin/python3 -m pip install --break-system-packages \
    faster-whisper pyaudio soundfile

# Pre-download model (optional but recommended)
echo "📥 Pre-downloading Whisper large-v3-turbo model..."
sudo -u $REAL_USER /usr/bin/python3 -c "
from faster_whisper import WhisperModel
print('Downloading large-v3-turbo...')
model = WhisperModel('large-v3-turbo', device='cpu', compute_type='int8')
print('Model cached successfully')
" || echo "Model will download on first use"

# #################################
# Kokoro TTS (Text-to-Speech)
# #################################

echo "📦 Installing Kokoro TTS..."
/usr/bin/python3 -m pip install --break-system-packages \
    kokoro sounddevice

# Also install Piper as fallback
/usr/bin/python3 -m pip install --break-system-packages piper-tts || true

# Download Piper voice model as backup
mkdir -p $USER_HOME/piper_voices
cd $USER_HOME/piper_voices
if [ ! -f "en_US-ryan-high.onnx" ]; then
    echo "📥 Downloading Piper Ryan voice..."
    wget -q "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/ryan/high/en_US-ryan-high.onnx" || true
    wget -q "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/ryan/high/en_US-ryan-high.onnx.json" || true
fi
chown -R $REAL_USER:$REAL_USER $USER_HOME/piper_voices

# #################################
# YOLO Vision
# #################################

echo "📦 Installing YOLO (ultralytics)..."
/usr/bin/python3 -m pip install --break-system-packages \
    ultralytics opencv-python

# #################################
# Ollama & Qwen 3
# #################################

echo "📦 Installing Ollama..."
curl -fsSL https://ollama.com/install.sh | sh

# Start Ollama service
systemctl enable ollama 2>/dev/null || true
systemctl start ollama 2>/dev/null || true

sleep 3

echo "📥 Pulling Qwen 3 model..."
sudo -u $REAL_USER ollama pull qwen3:8b || echo "Pull Qwen manually: ollama pull qwen3:8b"

# #################################
# Bash Aliases
# #################################

if ! grep -q "# I_CAN Robot Aliases" $USER_HOME/.bashrc; then
    cat >> $USER_HOME/.bashrc << 'EOF'

# I_CAN Robot Aliases
alias ros_source='source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash 2>/dev/null'
alias go2_sim='ros2 launch unitree_go2_sim unitree_go2_launch.py'
alias ign_gazebo='ign gazebo'
alias test_whisper='python3 ~/ros2_ws/src/ican_voice/test_whisper.py'
alias test_tts='python3 ~/ros2_ws/src/ican_voice/test_tts.py'

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
echo "✅ HUMBLE REMOTE INSTALL COMPLETE"
echo "=========================================="
echo ""
echo "Installed (SYSTEM Python for ROS2 compatibility):"
echo "  • ROS 2 Humble Desktop"
echo "  • Gazebo Ignition Fortress"
echo "  • PyTorch with CUDA 12.1"
echo "  • Faster-Whisper (large-v3-turbo) - GPU STT"
echo "  • Kokoro TTS (82M) - GPU TTS"
echo "  • Piper TTS (fallback)"
echo "  • YOLO (ultralytics) - Vision"
echo "  • Ollama + Qwen 3 - LLM"
echo "  • Chrome & VS Code"
echo ""
echo "Aliases:"
echo "  ros_source    - Source ROS 2 workspace"
echo "  go2_sim       - Launch simulation"
echo "  test_whisper  - Test speech recognition"
echo "  test_tts      - Test text-to-speech"
echo ""
echo "Test AI:"
echo "  ollama run qwen3:8b"
echo "  python3 -c \"from faster_whisper import WhisperModel; print('OK')\""
echo "  python3 -c \"import torch; print('CUDA:', torch.cuda.is_available())\""
echo ""
echo "⚠️  AI packages installed to SYSTEM Python (required for ros2 run)"
echo ""
echo "Restart terminal or: source ~/.bashrc"
echo "=========================================="