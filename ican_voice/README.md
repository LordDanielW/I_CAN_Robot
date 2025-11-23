# ican_voice - The Ears 🎤

This package provides speech-to-text capabilities for the I_CAN Robot using faster-whisper.

## Features

- **Real-time audio capture** using PyAudio
- **GPU-accelerated transcription** with faster-whisper
- **Threaded architecture** - audio processing doesn't block ROS 2
- **Voice Activity Detection (VAD)** - only transcribes when you speak
- **Configurable parameters** - adjust model size, thresholds, etc.

## Installation

### 1. System Dependencies (PortAudio for PyAudio)

```bash
sudo apt-get update
sudo apt-get install -y portaudio19-dev python3-pyaudio
```

### 2. Python Dependencies (in your venv)

Make sure your virtual environment is activated:

```bash
source ~/ros2_mcp_env/bin/activate
```

Install the required Python packages:

```bash
pip install pyaudio numpy faster-whisper
```

### 3. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select ican_voice
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch ican_voice voice_node.launch.py
```

### With Custom Parameters

```bash
# Use a smaller/faster model
ros2 launch ican_voice voice_node.launch.py model_size:=tiny.en

# Force CPU usage
ros2 launch ican_voice voice_node.launch.py device:=cpu

# Adjust sensitivity (lower = more sensitive)
ros2 launch ican_voice voice_node.launch.py energy_threshold:=300.0
```

### Run Without Launch File

```bash
ros2 run ican_voice voice_node
```

## Testing the Node

In another terminal, listen to the published speech:

```bash
ros2 topic echo /human/speech
```

Speak into your microphone - you should see your words appear on the topic!

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_size` | `base.en` | Whisper model: tiny.en, base.en, small.en, medium.en, large-v2 |
| `device` | `auto` | Compute device: auto, cuda, or cpu |
| `compute_type` | `int8` | Precision: int8 (fastest), float16, float32 (most accurate) |
| `mic_device_index` | `-1` | Microphone device (-1 for default) |
| `sample_rate` | `16000` | Audio sample rate in Hz |
| `energy_threshold` | `500.0` | RMS energy threshold for speech detection |
| `silence_duration` | `1.5` | Seconds of silence to end a phrase |
| `min_phrase_duration` | `0.5` | Minimum phrase length in seconds |

## Topics

### Published

- `/human/speech` (`std_msgs/String`) - Transcribed speech text

## Troubleshooting

### WSL2 Microphone Access

WSL2 doesn't natively support audio input. You have a few options:

1. **WSLg with PulseAudio** (recommended for WSL2)
   ```bash
   sudo apt-get install pulseaudio
   pulseaudio --start
   ```

2. **Use Windows microphone via PulseAudio network**
   - Set up PulseAudio on Windows to accept network connections
   - Configure WSL to connect to Windows PulseAudio server

3. **Use a USB audio device**
   - Attach USB microphone to WSL
   - Check with: `lsusb`

### List Available Microphones

```python
python3 -c "import pyaudio; pa = pyaudio.PyAudio(); [print(f'{i}: {pa.get_device_info_by_index(i)[\"name\"]}') for i in range(pa.get_device_count())]; pa.terminate()"
```

### Model Downloads

On first run, faster-whisper will download the model to `~/.cache/huggingface/`.

This may take a few minutes depending on model size:
- tiny.en: ~75 MB
- base.en: ~145 MB  
- small.en: ~488 MB
- medium.en: ~1.5 GB

### No Audio Detected

If the node runs but doesn't detect speech:

1. Check your microphone is working: `arecord -d 5 test.wav`
2. Lower the `energy_threshold` parameter
3. Verify PyAudio can access your mic (see "List Available Microphones" above)

## Architecture Notes

The voice node uses a **threaded design**:

- **Main Thread**: Runs the ROS 2 node spin loop
- **Audio Thread**: Continuously captures and processes audio

This ensures the ROS 2 heartbeat/spin is never blocked by audio processing.

## Next Steps

Once the ears are working, you'll connect them to the brain (ican_orchestrator) which will process the speech and decide what to do.
