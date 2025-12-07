# ROS2 with Python Virtual Environment Setup

## Problem
When running ROS2 nodes that require Python packages not available in the system Python (like `ollama`, `vosk`, `pyttsx3`), you'll encounter `ModuleNotFoundError` even if the packages are installed in a virtual environment.

## Root Cause
ROS2's `colcon build` creates entry point scripts that use the system Python interpreter (`#!/usr/bin/python3`), not the virtual environment's Python. When you run `ros2 run <package> <node>`, it executes these scripts with the system Python, which doesn't have access to packages installed in your virtual environment.

## Solution
Install required Python packages in the **system Python** using the `--break-system-packages` flag (or `--user` flag for user-level installation).

## Step-by-Step Instructions

### 1. Identify Required Packages
Check your ROS2 package's `setup.py` to see what Python dependencies are needed:
```python
install_requires=['setuptools', 'ollama', 'vosk', 'pyttsx3']
```

### 2. Install Packages in System Python
```bash
# Deactivate any virtual environment first
deactivate

# Install packages to system Python (user installation)
/usr/bin/python3 -m pip install --user ollama vosk pyttsx3 pyaudio

# OR use --break-system-packages if --user doesn't work
/usr/bin/python3 -m pip install --break-system-packages ollama vosk pyttsx3 pyaudio
```

### 3. Build Your ROS2 Workspace
```bash
cd ~/ros2_ws
colcon build --packages-select ican_brain ican_voice --symlink-install
```

### 4. Source and Run
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ican_brain ollama_node
ros2 run ican_voice vosk_node
ros2 run ican_voice tts_node
```

## Setting Up on a New Machine

### Prerequisites
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 dependencies
sudo apt install python3-pip python3-dev portaudio19-dev

# Install audio libraries for vosk and pyttsx3
sudo apt install python3-pyaudio espeak espeak-ng libespeak-dev
```

### Install Python Packages
```bash
# Install required packages to system Python
/usr/bin/python3 -m pip install --user ollama vosk pyttsx3 pyaudio SpeechRecognition faster-whisper ultralytics opencv-python torch torchvision

# IMPORTANT: Remove user-installed numpy (it conflicts with ROS2 cv_bridge)
# The system numpy 1.26.4 works with all packages
rm -rf ~/.local/lib/python3.12/site-packages/numpy*
```

### Clone and Build Workspace
```bash
# Clone your repository
cd ~
git clone <your-repo-url> ros2_ws/src/I_CAN_Robot

# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

### Download Vosk Model (for offline speech recognition)
```bash
# Download a vosk model
cd ~
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
mv vosk-model-small-en-us-0.15 vosk-model
```

### Verify Ollama Installation
```bash
# Install Ollama (if not already installed)
curl -fsSL https://ollama.com/install.sh | sh

# Start Ollama service
ollama serve &

# Pull the model
ollama pull qwen2.5:7b
```

## Testing Individual Components

### Test Ollama
```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_brain/ican_brain
python3 test_ollama.py
```

### Test Vosk
```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_voice/ican_voice
python3 test_vosk.py
```

### Test pyttsx3
```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_voice/ican_voice
python3 test_tts.py
```

### Test Whisper
```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_voice/ican_voice
python3 test_whisper.py
```

### Test YOLO
```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_see/ican_see
python3 test_yolo.py
```

## Running the Full System

### Terminal 1: Ollama Node
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ican_brain ollama_node
```

### Terminal 2: Voice Recognition Node
```bash
source ~/ros2_ws/install/setup.bash
# Choose one:
ros2 run ican_voice vosk_node          # Vosk with direct microphone
ros2 run ican_voice vosk_server_node   # Vosk with audio stream
ros2 run ican_voice whisper_server_node # Whisper with audio stream
```

### Terminal 3: Text-to-Speech Node
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ican_voice tts_node
```

### Terminal 4: Chat Node (sends prompts)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ican_brain ollama_chat
```

## Troubleshooting

### ModuleNotFoundError
If you get `ModuleNotFoundError: No module named 'xxx'`:
```bash
# Verify package is installed
/usr/bin/python3 -c "import xxx; print(xxx.__version__)"

# If not, install it
/usr/bin/python3 -m pip install --user xxx
```

### Rebuild After Installing Packages
```bash
cd ~/ros2_ws
colcon build --packages-select <package-name> --symlink-install
```

### Check Python Path
```bash
# See which Python ROS2 is using
head -1 ~/ros2_ws/install/<package>/lib/<package>/<node>
# Should show: #!/usr/bin/python3
```

## Why Not Use Virtual Environments?
While virtual environments are great for development, ROS2's build system (colcon) generates entry point scripts that hardcode the system Python path. You would need to:
1. Modify every generated script's shebang line, or
2. Rebuild with a custom Python interpreter path, or
3. Use system Python packages (recommended approach)

The system installation approach is simpler and more reliable for ROS2 deployment.
