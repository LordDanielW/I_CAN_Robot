# Quick Setup Prompt for New Machine

Copy and paste these commands on a new Ubuntu machine with ROS2 installed:

```bash
# ====================================
# 1. INSTALL SYSTEM DEPENDENCIES
# ====================================
sudo apt update
sudo apt install -y python3-pip python3-dev portaudio19-dev python3-pyaudio espeak espeak-ng libespeak-dev alsa-utils

# ====================================
# 2. INSTALL PYTHON PACKAGES
# ====================================
/usr/bin/python3 -m pip install --user ollama vosk pyttsx3 pyaudio SpeechRecognition faster-whisper ultralytics opencv-python

# IMPORTANT: Remove user numpy if it was installed (causes conflicts with cv_bridge)
rm -rf ~/.local/lib/python3.12/site-packages/numpy*

# ====================================
# 3. INSTALL AND SETUP OLLAMA
# ====================================
curl -fsSL https://ollama.com/install.sh | sh
ollama serve > /dev/null 2>&1 &
sleep 5
ollama pull qwen2.5:7b

# ====================================
# 4. DOWNLOAD VOSK MODEL
# ====================================
cd ~
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
mv vosk-model-small-en-us-0.15 vosk-model
rm vosk-model-small-en-us-0.15.zip

# ====================================
# 5. CLONE AND BUILD ROS2 WORKSPACE
# ====================================
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
# Replace with your actual repository URL
git clone https://github.com/LordDanielW/I_CAN_Robot.git

cd ~/ros2_ws
colcon build --symlink-install

# ====================================
# 6. SOURCE WORKSPACE
# ====================================
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# ====================================
# 7. TEST INDIVIDUAL COMPONENTS
# ====================================
echo "Testing Ollama..."
cd ~/ros2_ws/src/I_CAN_Robot/ican_orchestrator/ican_orchestrator
python3 test_ollama.py

echo "Testing TTS..."
cd ~/ros2_ws/src/I_CAN_Robot/ican_voice/ican_voice
python3 test_tts.py

echo "Testing Vosk..."
python3 test_vosk.py

echo "Testing Whisper..."
python3 test_whisper.py

echo "Testing YOLO..."
cd ~/ros2_ws/src/I_CAN_Robot/ican_see/ican_see
python3 test_yolo.py

# ====================================
# 8. DONE! NOW RUN YOUR NODES
# ====================================
echo "Setup complete! You can now run:"
echo "  ros2 run ican_orchestrator ollama_node"
echo "  ros2 run ican_voice vosk_node"
echo "  ros2 run ican_voice vosk_server_node"
echo "  ros2 run ican_voice whisper_server_node"
echo "  ros2 run ican_voice tts_node"
echo "  ros2 run ican_orchestrator ollama_chat"
echo "  ros2 run ican_see yolo_server_node"
echo ""
echo "Test vision with camera:"
echo "  ros2 launch ican_bringup local_nodes.launch.py"
```

## Manual Testing Commands

### Test Ollama Node
```bash
ros2 run ican_orchestrator ollama_node
# In another terminal:
ros2 topic pub /llm_prompt std_msgs/String "data: 'Hello, tell me a joke'"
ros2 topic echo /llm_response
```

### Test Vosk Node
```bash
ros2 run ican_voice vosk_node
# Speak into microphone, check output:
ros2 topic echo /speech_text
```

### Test TTS Node
```bash
ros2 run ican_voice tts_node
# In another terminal:
ros2 topic pub /robot_speech std_msgs/String "data: 'Hello, I am a robot'"
```

### Test Complete Pipeline
```bash
# Terminal 1: Start all nodes with launch file (if available)
ros2 launch ican_bringup robot.launch.py

# OR start each node separately:
# Terminal 1:
ros2 run ican_orchestrator ollama_node

# Terminal 2:
ros2 run ican_voice vosk_node

# Terminal 3:
ros2 run ican_voice tts_node

# Terminal 4: Monitor topics
ros2 topic list
ros2 topic echo /llm_response
```

## Troubleshooting

### If Ollama won't start:
```bash
systemctl status ollama  # Check if running as service
ollama serve &           # Start manually
ollama list              # List downloaded models
```

### If microphone doesn't work:
```bash
arecord -l              # List recording devices
alsamixer               # Adjust microphone volume
```

### If TTS doesn't work:
```bash
espeak "test"           # Test espeak directly
speaker-test            # Test audio output
```

### If Python packages not found:
```bash
/usr/bin/python3 -c "import ollama; print('OK')"
/usr/bin/python3 -c "import vosk; print('OK')"
/usr/bin/python3 -c "import pyttsx3; print('OK')"
```

### Rebuild workspace:
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```
