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
| **go2_simple_nav** | Navigation and vision-language nodes (joystick_goal_webrtc, qwen3_vl_node, webcam streaming) | [go2_simple_nav/README.md](go2_simple_nav/README.md) |

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
- **LLM:** Ollama with Qwen3-VL:8B model

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
ollama pull qwen3-vl:8b
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

### Joystick/WebRTC Navigation
```bash
# Start joystick-based navigation (remote)
ros2 launch go2_simple_nav joystick_goal_webrtc.launch.py
```
This node allows remote goal setting via joystick or web interface, sending high-level navigation commands to the Go2 robot.

### Vision-Language Inference (Qwen3-VL:8B)
```bash
# Start vision-language node
ros2 launch go2_simple_nav qwen3_vl.launch.py
```
Trigger the node by publishing to the `trigger_capture` topic. The node will capture an image and run Qwen3-VL:8B inference, publishing results to `vlm_output`.

```bash
# Test script for publishing a simple image feed
ros2 run go2_simple_nav publish_webcam_image.py
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
| `/vlm_output` | String | Vision-language model output from Qwen3-VL:8B node |
| `/camera/image_raw` | sensor_msgs/Image | Raw webcam images for VL/vision nodes |
| `/trigger_capture` | Bool | Triggers Qwen3-VL:8B node to process an image |

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
Default: **qwen3-vl:8b**

Change model:
```bash
ros2 run ican_brain ollama_tool_node --ros-args -p llm_model:='llama3:8b'
```

## Navigation and Vision-Language Nodes

### Navigation (go2_simple_nav)

The `go2_simple_nav` package provides lightweight navigation for the Unitree Go2 robot:

- **joystick_goal_webrtc**: Node for setting navigation goals via joystick input and WebRTC, enabling teleoperation and remote goal setting through a web interface.

#### Launch Files

- `joystick_goal_webrtc.launch.py`: Launches the joystick_goal_webrtc node for remote navigation control.

#### Example Usage

**Start joystick-based navigation (remote):**
```bash
ros2 launch go2_simple_nav joystick_goal_webrtc.launch.py
```

---

### Vision-Language Model (VLM/LLM)

The vision-language model (VLM) is handled by the same LLM system (Qwen3-VL:8B) used for language tasks. The `qwen3_vl_node` enables multimodal (image+text) inference, allowing the robot to interpret visual scenes and answer questions about its environment.

- **qwen3_vl_node**: Node for Qwen3-VL:8B inference (image+text via LLM)

#### Launch Files

- `qwen3_vl.launch.py`: Launches the Qwen3-VL:8B node for vision-language inference.

#### Example Usage

**Start vision-language node:**
```bash
ros2 launch go2_simple_nav qwen3_vl.launch.py
```

Trigger the node by publishing to the `trigger_capture` topic. The node will capture an image and run Qwen3-VL:8B inference, publishing results to `vlm_output`.

See `go2_simple_nav/README.md` for more details and configuration options.

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
├── go2_simple_nav/    # Navigation, joystick, and vision-language nodes
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

**New Navigation or Vision-Language Feature:**
- Use `go2_simple_nav` for joystick/WebRTC navigation (`joystick_goal_webrtc`) and vision-language inference (`qwen3_vl_node`, Qwen3-VL:8B).
- Publish images to `/camera/image_raw` for VL/vision nodes.
- Trigger Qwen3-VL:8B by publishing `Bool` to `/trigger_capture`; results on `/vlm_output`.
- YOLO detections available at `/yolo_detections`.
- Integrate vision or navigation context with LLM prompts as needed.

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
ollama run qwen3-vl:8b "Hello"
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
- **Qwen3-VL:8B:** https://ollama.com/library/qwen3-vl
- **YOLOv8:** https://github.com/ultralytics/ultralytics
- **Whisper:** https://github.com/openai/whisper

## Contributing

See individual package READMEs for development details.

## License

[Add license information]

## Authors

- LordDanielW - [GitHub](https://github.com/LordDanielW)
- Blaine Oania - [GitHub](https://github.com/BlaineKTMO)

---

**Status:** ✅ Active Development

For detailed package documentation, see individual package READMEs linked above.

## Starting the Go2 Robot

This section describes how to power on and connect to the Unitree Go2 robot, including controller and UWB setup, and configuring the SDK environment variables for ROS2 integration.

### 1. Power On the Go2
- Double press and hold the power button on the Go2 until you hear the startup sound and see the status lights.
- Wait for the robot to finish its boot sequence.

### 2. Start the Controller
- Double press and hold to power on the Unitree controller (handheld remote).
- Wait for it to connect to the robot (status icon should indicate connection).

### 3. (Optional) Start the UWB Controller
- If using the UWB (Ultra-Wideband) positioning system, press the power button on the UWB controller and ensure it is paired with the robot.
- Follow Unitree's UWB pairing instructions if needed.

### 4. Connect to the Robot (Ethernet or WiFi)
- **Ethernet:**
  - Connect your computer directly to the Go2's Ethernet port using a standard cable.
  - Set your computer's IP to be on the same subnet as the robot (e.g., `192.168.123.10`).
  - The default robot IP is usually `192.168.123.161` (check your Go2 docs or display).
- **WiFi:**
  - Connect your computer to the same WiFi network as the Go2.
  - Find the robot's IP address from the application.

### 5. Set Environment Variables for the Go2 SDK
The unofficial Go2 SDK requires two environment variables to be set before running any SDK-based nodes:

```bash
export robot_ip=192.168.123.161   # Replace with your Go2's actual IP
export conn_type=webrtc
```
- `robot_ip`: The IP address of the Go2 robot.
- `conn_type`: Connection type. Use `webrtc` for webrtc, `cyclonedds` for cyclone dds.

You can add these lines to your `~/.bashrc` or run them in each terminal before launching ROS2 nodes that use the Go2 SDK.

### 6. Launch the Go2 Robot Software

After setting the environment variables, launch the Go2 robot system using the provided launch file from the unofficial SDK:

```bash
# Example (adjust path as needed)
ros2 launch go2_robot_sdk robot.launch.py \
  rviz2:=true \
  nav2:=true \
  slam:=true \
  foxglove:=true \
  joystick:=true \
  teleop:=true
```

**Parameters:**
- `rviz2` (default: true): Launch RViz2 visualization
- `nav2` (default: true): Launch Nav2 navigation stack
- `slam` (default: true): Launch SLAM toolbox
- `foxglove` (default: true): Launch Foxglove Bridge
- `joystick` (default: true): Launch joystick node
- `teleop` (default: true): Launch teleoperation node

## Submodules

### go2_ros2_sdk
This repository includes the [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) as a git submodule in the `go2_ros2_sdk` directory. This SDK provides ROS2 integration and drivers for the Unitree Go2 robot, including:
- High-level control nodes and message definitions
- WebRTC and Ethernet communication support
- Example launch files and configuration for Go2

#### Initializing/Updating Submodules
If you clone this repository, make sure to initialize and update submodules:
```bash
git submodule update --init --recursive
```

#### Building the SDK
Refer to the SDK's own README for build and usage instructions. Typical usage involves building with colcon:
```bash
cd go2_ros2_sdk
colcon build --symlink-install
```
