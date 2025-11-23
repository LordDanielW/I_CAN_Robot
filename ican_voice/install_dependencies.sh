#!/bin/bash
# Installation script for ican_voice dependencies

set -e

echo "=========================================="
echo "Installing ican_voice Dependencies"
echo "=========================================="

# Check if running in WSL
if grep -qi microsoft /proc/version; then
    echo "Detected WSL environment"
fi

# Install system dependencies
echo ""
echo "Installing system dependencies (requires sudo)..."
sudo apt-get update
sudo apt-get install -y portaudio19-dev python3-pyaudio

# Check if venv is activated
if [[ -z "${VIRTUAL_ENV}" ]]; then
    echo ""
    echo "WARNING: No virtual environment detected!"
    echo "It's recommended to activate your venv first:"
    echo "  source ~/ros2_mcp_env/bin/activate"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Install Python packages
echo ""
echo "Installing Python packages..."
pip install --upgrade pip
pip install pyaudio numpy faster-whisper

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Build the package:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select ican_voice"
echo "   source install/setup.bash"
echo ""
echo "2. Test the microphone:"
echo "   python3 -c 'import pyaudio; pa = pyaudio.PyAudio(); print(f\"Found {pa.get_device_count()} audio devices\"); pa.terminate()'"
echo ""
echo "3. Launch the voice node:"
echo "   ros2 launch ican_voice voice_node.launch.py"
echo ""
