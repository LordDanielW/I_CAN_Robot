#!/bin/bash
# Test script for voice_node

echo "=========================================="
echo "Testing Voice Node Installation"
echo "=========================================="
echo ""

# Activate virtual environment
source ~/ros2_mcp_env/bin/activate

# Source ROS workspace
cd ~/ros2_ws
source install/setup.bash

echo "✓ Virtual environment activated"
echo "✓ ROS workspace sourced"
echo ""

# Check Python imports
echo "Checking Python dependencies..."
python3 -c "import pyaudio; print('  ✓ pyaudio')" || echo "  ✗ pyaudio FAILED"
python3 -c "import numpy; print('  ✓ numpy')" || echo "  ✗ numpy FAILED"
python3 -c "import faster_whisper; print('  ✓ faster-whisper')" || echo "  ✗ faster-whisper FAILED"
python3 -c "import rclpy; print('  ✓ rclpy')" || echo "  ✗ rclpy FAILED"
echo ""

# Check if voice_node executable exists
echo "Checking voice_node installation..."
if ros2 pkg executables ican_voice | grep -q voice_node; then
    echo "  ✓ voice_node executable found"
else
    echo "  ✗ voice_node executable NOT found"
fi
echo ""

# Check audio devices
echo "Checking audio devices..."
DEVICE_COUNT=$(python3 -c "import pyaudio; pa = pyaudio.PyAudio(); print(pa.get_device_count()); pa.terminate()" 2>/dev/null)
if [ "$DEVICE_COUNT" -gt 0 ]; then
    echo "  ✓ Found $DEVICE_COUNT audio device(s)"
    python3 -c "import pyaudio; pa = pyaudio.PyAudio(); [print(f'    {i}: {pa.get_device_info_by_index(i)[\"name\"]}') for i in range(pa.get_device_count())]; pa.terminate()" 2>/dev/null
else
    echo "  ⚠ No audio devices found (expected in WSL2)"
    echo "    See WSL2_AUDIO_SETUP.md for solutions"
fi
echo ""

echo "=========================================="
echo "Installation Test Complete"
echo "=========================================="
echo ""
echo "To test the node without audio input:"
echo ""
echo "  Terminal 1 - Start the node:"
echo "    cd ~/ros2_ws"
echo "    source install/setup.bash"
echo "    source ~/ros2_mcp_env/bin/activate"
echo "    ros2 run ican_voice voice_node"
echo ""
echo "  Terminal 2 - Simulate speech:"
echo "    cd ~/ros2_ws"
echo "    source install/setup.bash"
echo "    ros2 topic pub /human/speech std_msgs/msg/String \"data: 'Hello robot'\" --once"
echo ""
echo "  Terminal 3 - Monitor the topic:"
echo "    cd ~/ros2_ws"
echo "    source install/setup.bash"
echo "    ros2 topic echo /human/speech"
echo ""
