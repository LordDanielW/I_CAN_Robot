# ican_voice - Speech Recognition & Text-to-Speech

Voice input and output capabilities for the I_CAN Robot using OpenAI Whisper and pyttsx3.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     ican_voice Package                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────────┐         ┌──────────────────┐         │
│  │ Audio Streamer   │────────>│ Whisper Server   │         │
│  │   (microphone)   │ /audio  │ (speech-to-text) │         │
│  │                  │ _stream │                  │         │
│  └──────────────────┘         └─────────┬────────┘         │
│                                          │                  │
│                                          v                  │
│                                    /speech_text             │
│                                          │                  │
│                                          v                  │
│                                 ┌────────────────┐          │
│           /robot_speech ───────>│   TTS Node     │          │
│                                 │  (pyttsx3)     │          │
│                                 └────────────────┘          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Nodes

### 1. audio_streamer_node
- **Purpose**: Captures microphone audio and publishes raw audio stream
- **Publishes**: `/audio_stream` (UInt8MultiArray) - Raw PCM audio bytes
- **Use**: Run on robot or local machine with microphone

### 2. whisper_server_node
- **Purpose**: Transcribes audio stream to text using OpenAI Whisper
- **Subscribes**: `/audio_stream` (UInt8MultiArray)
- **Publishes**: `/speech_text` (String) - Transcribed speech
- **Parameters**:
  - `model_size`: tiny.en, base.en, small.en, medium.en (default: base.en)
  - `device`: cpu or cuda (default: cpu)
  - `compute_type`: int8, float16, float32 (default: int8)

### 3. tts_node
- **Purpose**: Speaks text aloud using pyttsx3 text-to-speech
- **Subscribes**: `/robot_speech` (String)
- **Use**: Robot verbal responses

## Quick Start

### Run Complete Voice System
```bash
# Terminal 1: Audio capture (on robot/local machine)
ros2 run ican_voice audio_streamer_node

# Terminal 2: Speech recognition (can run remotely)
ros2 run ican_voice whisper_server_node

# Terminal 3: Text-to-speech output
ros2 run ican_voice tts_node

# Terminal 4: Monitor recognized speech
ros2 topic echo /speech_text
```

### Test Without Microphone
```bash
# Simulate speech input
ros2 topic pub /speech_text std_msgs/String "data: 'Hello robot'" --once

# Test TTS output
ros2 topic pub /robot_speech std_msgs/String "data: 'I am ready'" --once
```

## Installation

See [Robot_Remote_Install.sh](../Robot_Remote_Install.sh) for full installation script.

### System Dependencies
```bash
sudo apt install -y portaudio19-dev python3-pyaudio espeak espeak-ng libespeak-dev
```

### Python Packages (System Python)
```bash
# Important: Install to system Python for ROS2 compatibility
/usr/bin/python3 -m pip install --break-system-packages \
    pyaudio faster-whisper pyttsx3
```

**Critical**: See [VENV_README.md](VENV_README.md) for why we use system Python with ROS2.

## Testing

### Test Whisper (No ROS)
```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_voice/ican_voice
python3 test_whisper.py
# Records 5 seconds, transcribes with Whisper
```

### Test TTS (No ROS)
```bash
python3 test_tts.py
# Speaks test message
```

## Parameters Reference

### whisper_server_node
| Parameter | Default | Options | Description |
|-----------|---------|---------|-------------|
| model_size | base.en | tiny.en, base.en, small.en, medium.en | Model size vs accuracy tradeoff |
| device | cpu | cpu, cuda | Compute device |
| compute_type | int8 | int8, float16, float32 | Precision (int8 fastest) |

### Example with parameters
```bash
ros2 run ican_voice whisper_server_node --ros-args \
  -p model_size:=tiny.en \
  -p device:=cpu \
  -p compute_type:=int8
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/audio_stream` | UInt8MultiArray | Published | Raw PCM audio (16kHz, mono, 16-bit) |
| `/speech_text` | String | Published | Transcribed speech from Whisper |
| `/robot_speech` | String | Subscribed | Text for robot to speak |

## Integration Example

Voice-controlled robot with wake word detection:

```bash
# Full system with prompt node and LLM
ros2 launch ican_bringup remote_nodes.launch.py
```

This launches:
1. whisper_server_node - Speech recognition
2. prompt_node - Wake word detection ("Hey Spot")
3. ollama_tool_node - LLM brain
4. tool_manager_node - Tool orchestration
5. dice_service_node - Example tool

## Performance Notes

- **tiny.en**: Fastest, good for real-time on CPU
- **base.en**: Balanced speed/accuracy (recommended)
- **small.en**: Better accuracy, slower
- **medium.en**: Best accuracy, requires GPU

Model downloads automatically on first run to `~/.cache/huggingface/hub/`.

## Troubleshooting

### No audio devices (WSL2)
WSL2 doesn't support audio natively. Options:
1. Use PulseAudio forwarding from Windows
2. Run audio_streamer_node on Windows, whisper on WSL2
3. Use network ROS2 to bridge audio topics

### Whisper model download slow
First run only. Models cached locally. Use `tiny.en` for fastest download.

### TTS not working
```bash
# Install espeak
sudo apt install espeak espeak-ng libespeak-dev

# Test
echo "hello" | espeak
```

### Module not found errors
See [VENV_README.md](VENV_README.md) - packages must be in system Python, not venv.

## Files

- `audio_streamer_node.py` - Microphone capture
- `whisper_server_node.py` - Speech-to-text
- `tts_node.py` - Text-to-speech
- `test_whisper.py` - Standalone Whisper test
- `test_tts.py` - Standalone TTS test
- `VENV_README.md` - **Critical**: ROS2 + Python package setup guide

## See Also

- [Voice Tool Calling Guide](../../VOICE_TOOL_CALLING_GUIDE.md) - Full system integration
- [ican_brain](../ican_brain/) - LLM brain and prompt processing
- [ican_tools](../ican_tools/) - Tool management system
