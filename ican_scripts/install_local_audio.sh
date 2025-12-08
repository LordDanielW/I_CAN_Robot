#!/bin/bash
# #################################
# Audio Streaming Dependencies Only
# For Ubuntu 22.04 (Humble)
# Installs: PyAudio, PortAudio drivers
# Does NOT install: AI models, Piper, Whisper, heavy TTS
# Use Case: Remote ROS2 node handling audio I/O with BLE devices
# #################################

set -e

# Check if script is run as root/sudo
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: This script must be run with sudo"
    echo "Usage: sudo bash $0"
    exit 1
fi

# Identify the real user
REAL_USER=${SUDO_USER:-$(whoami)}
USER_HOME="/home/$REAL_USER"

echo "=========================================="
echo "Installing Audio Streaming Dependencies"
echo "Target: Ubuntu 22.04 (ROS2 Humble)"
echo "User: $REAL_USER"
echo "=========================================="

# #################################
# System Audio Drivers
# #################################

echo "📦 Installing PortAudio system libraries..."
apt update
apt install -y \
    portaudio19-dev \
    python3-pyaudio \
    libasound2-dev \
    alsa-utils \
    pulseaudio-utils

echo "✅ Audio drivers installed"

# #################################
# Python Audio Libraries
# #################################

echo "📦 Installing Python audio packages..."

# Check if we're in a ROS2 workspace with a venv
VENV_PATH="$USER_HOME/ros2_dev_env"

if [ -d "$VENV_PATH" ]; then
    echo "   -> Found virtual environment at $VENV_PATH"
    echo "   -> Installing PyAudio in venv..."
    sudo -u $REAL_USER $VENV_PATH/bin/pip install pyaudio
else
    echo "   -> No venv found, installing system-wide..."
    pip3 install pyaudio
fi

echo "✅ Python audio packages installed"

# #################################
# Audio Device Test
# #################################

echo ""
echo "=========================================="
echo "🎤 Testing Audio Devices"
echo "=========================================="

echo "Available audio input devices:"
arecord -l 2>/dev/null || echo "   -> No input devices found (may need BLE connection)"

echo ""
echo "Available audio output devices:"
aplay -l 2>/dev/null || echo "   -> No output devices found (may need BLE connection)"

# #################################
# Final Instructions
# #################################

echo ""
echo "=========================================="
echo "✅ Installation Complete!"
echo "=========================================="
echo ""
echo "📋 Next Steps:"
echo "   1. Connect your BLE microphone/speaker"
echo "   2. Verify devices with: arecord -l && aplay -l"
echo "   3. Rebuild ROS2 workspace:"
echo "      cd ~/ros2_ws"
echo "      colcon build --packages-select ican_voice"
echo "      source install/setup.bash"
echo "   4. Test audio streaming:"
echo "      ros2 run ican_voice audio_streamer_node"
echo ""
echo "🔧 Troubleshooting BLE Audio:"
echo "   - Check PulseAudio: pactl list sinks short"
echo "   - Check ALSA: aplay -L"
echo "   - Test microphone: arecord -d 3 test.wav"
echo "   - Test speaker: aplay test.wav"
echo ""
