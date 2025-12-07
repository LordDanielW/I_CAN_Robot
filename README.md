# I_CAN Robot

**Intelligent Context Aware Navigated Robot**

A voice-controlled quadruped robot with vision, LLM-based decision making, and ROS2-native tool calling.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        I_CAN Robot System                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Microphone → Whisper → Prompt Node → Ollama LLM               │
│                            ↓              ↓                     │
│                       Wake Word      Tool Calling               │
│                       Detection                                 │
│                            ↓              ↓                     │
│                       Tool Manager ← Tool Results               │
│                            ↓                                    │
│                    Tool Nodes (Dice, Movement, etc.)            │
│                                                                 │
│  Camera → YOLO → Object Detection → Vision Context             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Quick Start

### Build Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch System

**Remote Computer (Powerful - AI inference):**
```bash
ros2 launch ican_bringup remote_nodes.launch.py
```

**Local Computer (Robot - Sensors/Actuators):**
```bash
ros2 launch ican_bringup local_nodes.launch.py
```

## ROS2 Packages

### Core Packages

| Package | Description | README |
|---------|-------------|--------|
| **ican_brain** | AI orchestration with Ollama LLM, wake word detection, tool calling | [ican_brain/README.md](ican_brain/README.md) |
| **ican_voice** | Speech recognition (Whisper) and text-to-speech (pyttsx3) | [ican_voice/README.md](ican_voice/README.md) |
| **ican_see** | Object detection with YOLOv8/v13 | [ican_see/README.md](ican_see/README.md) |
| **ican_tools** | ROS2-native tool registry and execution system | [ican_tools/README.md](ican_tools/README.md) |

### Hardware Integration Packages

| Package | Description |
|---------|-------------|
| **ican_bringup** | Launch files for complete system |
| **ican_description** | URDF robot model |
| **ican_gazebo** | Gazebo simulation |

## System Requirements

### Software
- **OS:** Ubuntu 24.04 LTS
- **ROS2:** Jazzy Jalisco
- **Python:** 3.12 (system Python, not venv)
- **LLM:** Ollama with qwen2.5:7b model

### Hardware
- **Microphone:** Any USB/3.5mm microphone
- **Camera:** USB webcam or CSI camera
- **Compute:** 
  - Minimum: 4 cores, 8GB RAM (CPU inference)
  - Recommended: GPU with CUDA for faster YOLO/Whisper

## Installation

### System Dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  python3-pip \
  portaudio19-dev \
  python3-pyaudio \
  espeak espeak-ng libespeak-dev
```

### Ollama (LLM Engine)
```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen2.5:7b
```

### Python Packages
```bash
# Install to system Python (required for ROS2)
/usr/bin/python3 -m pip install --break-system-packages \
  ollama \
  faster-whisper \
  pyttsx3 \
  pyaudio \
  ultralytics \
  opencv-python \
  torch \
  torchvision
```

**Why system Python?** ROS2's `colcon build` generates entry points using `#!/usr/bin/python3`, which requires packages in system Python. See `ican_voice/VENV_README.md` for details.

### Clone and Build
```bash
# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/LordDanielW/I_CAN_Robot.git

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage Examples

### Voice-Controlled Dice Rolling
```
You: "Hey Spot, roll a d20"
Robot: "I rolled a d20 and got 17!"
```

**Flow:**
1. Whisper transcribes speech → `/speech_text`
2. Prompt node detects "Hey Spot" wake word
3. Ollama generates: "Rolling! TOOL:roll(d20)"
4. Tool manager calls dice service
5. Result published → TTS speaks response

### Object Detection
```bash
# View detections
ros2 topic echo /yolo_detections

# YOLO detects: person, chair, laptop with confidence scores
```

### Testing Individual Components

**Test Speech Recognition:**
```bash
ros2 run ican_voice whisper_server_node
ros2 topic echo /speech_text
```

**Test LLM:**
```bash
ros2 run ican_brain ollama_tool_node
ros2 topic pub /llm_prompt std_msgs/String "data: 'Hello Spot'"
ros2 topic echo /llm_response
```

**Test Vision:**
```bash
ros2 run image_tools cam2image --ros-args -p frequency:=5.0
ros2 run ican_see yolo_server_node
ros2 topic echo /yolo_detections
```

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/speech_text` | String | Transcribed speech from Whisper |
| `/llm_prompt` | String | Prompts sent to LLM |
| `/llm_response` | String | LLM responses with tool calls |
| `/tool_result` | String | Tool execution results |
| `/yolo_detections` | Detection2DArray | Object detections from YOLO |
| `/robot_speech` | String | Text for TTS to speak |

## Configuration

### Wake Word
Default: **"Hey Spot"**

Change in prompt_node:
```bash
ros2 run ican_brain prompt_node --ros-args -p wake_word:='hey robot'
```

### YOLO Model
Default: **YOLOv8n** (stable, auto-download)

Optional: **YOLOv13n** (+3% mAP, manual setup) - Edit `local_nodes.launch.py`

### Whisper Model
Default: **base.en** (balanced speed/accuracy)

Options: tiny.en, base.en, small.en, medium.en
```bash
ros2 run ican_voice whisper_server_node --ros-args -p model_size:=tiny.en
```

### LLM Model
Default: **qwen2.5:7b**

Change model:
```bash
ros2 run ican_brain ollama_tool_node --ros-args -p llm_model:='llama3:8b'
```

## Architecture Details

### Voice Pipeline
```
Microphone → audio_streamer → /audio_stream
                                    ↓
                            whisper_server → /speech_text
                                                  ↓
                                            prompt_node
                                            (wake word)
                                                  ↓
                                            /llm_prompt
                                                  ↓
                                          ollama_tool_node
                                                  ↓
                                           /llm_response
                                                  ↓
                                          tool_manager
                                                  ↓
                                         /tool_result
```

### Tool Calling System

**Format:** LLM embeds tool calls in responses
```
"I'll roll for you! TOOL:roll(d20)"
```

**Available Tools:**
- `roll(dice_notation)` - RPG dice rolling
- `roll_stats()` - Character ability scores
- `coin_flip(count)` - Coin flipping
- `roll_history(limit)` - Recent roll history

**Adding Tools:** See `ican_tools/README.md` for guide

### Vision Integration
- YOLOv8/v13 object detection (80 COCO classes)
- 5Hz camera rate (real-time robotics)
- GPU acceleration supported
- Detection results published to `/yolo_detections`

## Development

### Package Structure
```
I_CAN_Robot/
├── ican_brain/        # AI orchestration, LLM, wake word
├── ican_voice/        # Speech recognition, TTS
├── ican_see/          # YOLO vision
├── ican_tools/        # Tool registry and execution
├── ican_bringup/      # Launch files
├── ican_description/  # URDF models
├── ican_gazebo/       # Simulation
└── README.md          # This file
```

### Adding New Features

**New Tool:**
1. Create tool node (subscribe command topic, publish result)
2. Register in `tool_manager_node.py`
3. LLM automatically gets tool in prompts

**New Voice Command:**
- No code changes needed
- LLM learns from tool descriptions
- Wake word triggers listening

**New Vision Feature:**
- YOLO detections available at `/yolo_detections`
- Integrate with LLM prompts for visual context

## Troubleshooting

### No Voice Recognition
```bash
# Check microphone
arecord -l

# Test Whisper
cd ~/ros2_ws/src/I_CAN_Robot/ican_voice/ican_voice
python3 test_whisper.py
```

### LLM Not Responding
```bash
# Check Ollama service
pgrep ollama
ollama list

# Test manually
ollama run qwen2.5:7b "Hello"
```

### YOLO Not Detecting
```bash
# Check camera
ls /dev/video*

# Test YOLO standalone
cd ~/ros2_ws/src/I_CAN_Robot/ican_see/ican_see
python3 test_yolo.py
```

### Module Not Found
```bash
# Check if package in system Python
/usr/bin/python3 -c "import <module>"

# If missing, install
/usr/bin/python3 -m pip install --break-system-packages <module>
```

### Topics Not Connected
```bash
# List active topics
ros2 topic list

# Check topic data
ros2 topic echo <topic_name>

# View node graph
rqt_graph
```

## Resources

### Documentation
- [ican_brain README](ican_brain/README.md) - AI brain details
- [ican_voice README](ican_voice/README.md) - Voice system
- [ican_see README](ican_see/README.md) - Vision system
- [ican_tools README](ican_tools/README.md) - Tool calling

### External
- **ROS2 Jazzy:** https://docs.ros.org/en/jazzy/
- **Ollama:** https://ollama.com/
- **Qwen 2.5:** https://ollama.com/library/qwen2.5
- **YOLOv8:** https://github.com/ultralytics/ultralytics
- **Whisper:** https://github.com/openai/whisper

## Contributing

See individual package READMEs for development details.

## License

[Add license information]

## Authors

- LordDanielW - [GitHub](https://github.com/LordDanielW)

---

**Status:** ✅ Active Development

For detailed package documentation, see individual package READMEs linked above.
