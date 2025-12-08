# Audio System Test Guide

## Test Nodes Overview

Two test nodes for verifying the audio pipeline:

1. **test_audio_streamer** - Tests microphone capture (listens to /audio_stream and plays it back)
2. **test_audio_playback** - Tests speaker output (generates test tones and sends to /audio/tts_stream)

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select ican_voice
source install/setup.bash
```

## Test 1: Audio Streamer (Microphone → Playback)

**What it does**: Captures from microphone and immediately plays back what it hears (loopback test)

### Terminal 1: Run audio streamer
```bash
ros2 run ican_voice audio_streamer_node
```

### Terminal 2: Run test node
```bash
ros2 run ican_voice test_audio_streamer
```

**Expected behavior**:
- Speak into microphone
- You should hear yourself with slight delay
- Statistics printed every 5 seconds
- Test verifies: microphone capture, ROS topic communication, audio format

**Troubleshooting**:
- If no audio device found: Check `arecord -l` and set device_index parameter
- If no sound heard: Check speaker volume and audio output device
- If choppy: Audio queue filling up, check CPU load

## Test 2: Audio Playback (Test Tones → Speaker)

**What it does**: Generates test tones and sends to playback node

### Terminal 1: Run audio playback node
```bash
ros2 run ican_voice audio_playback_node
```

### Terminal 2: Run test node
```bash
ros2 run ican_voice test_audio_playback
```

**Interactive menu**:
```
Select test (1-4, q to quit): 
  1 - Play test tone (440Hz A4 note)
  2 - Play ascending tones (C major scale)
  3 - Play beep pattern (3 beeps)
  4 - Play white noise burst
  q - Quit
```

**Expected behavior**:
- Press 1-4 to play different test sounds
- Sounds should play through speakers
- Test verifies: audio generation, ROS topic communication, playback system

**Troubleshooting**:
- If no sound: Check playback_node is running and speaker volume
- If distorted: Try reducing volume parameter
- Wrong sample rate: Check audio_playback_node sample_rate parameter matches

## Test 3: Full Pipeline Test

Test complete audio chain: Microphone → Whisper → TTS → Speaker

### Terminal 1: Audio streamer
```bash
ros2 run ican_voice audio_streamer_node
```

### Terminal 2: Whisper server
```bash
ros2 run ican_voice whisper_server_node
```

### Terminal 3: TTS node
```bash
ros2 run ican_voice tts_node
```

### Terminal 4: Audio playback
```bash
ros2 run ican_voice audio_playback_node
```

### Terminal 5: Monitor (optional)
```bash
# Watch speech recognition
ros2 topic echo /speech_text

# Watch audio stream
ros2 topic hz /audio_stream
ros2 topic hz /audio/tts_stream
```

## Parameters

### test_audio_streamer
```bash
ros2 run ican_voice test_audio_streamer --ros-args \
    -p sample_rate:=16000 \
    -p channels:=1 \
    -p buffer_size:=1024
```

### test_audio_playback
```bash
ros2 run ican_voice test_audio_playback --ros-args \
    -p sample_rate:=24000 \
    -p test_frequency:=440.0 \
    -p duration:=1.0 \
    -p volume:=0.3
```

## Debugging

### Check topics
```bash
ros2 topic list
ros2 topic info /audio_stream
ros2 topic info /audio/tts_stream
```

### Monitor audio data rate
```bash
ros2 topic hz /audio_stream
ros2 topic bw /audio_stream
```

### Check for audio devices
```bash
# List recording devices
arecord -l

# List playback devices
aplay -l

# Test recording
arecord -d 5 test.wav

# Test playback
aplay test.wav
```

### Python audio test
```python
import pyaudio
p = pyaudio.PyAudio()

# List devices
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    print(f"{i}: {info['name']}, In:{info['maxInputChannels']}, Out:{info['maxOutputChannels']}")
```

## Common Issues

### Issue: "OSError: [Errno -9997] Invalid sample rate"
**Solution**: Your audio device doesn't support the sample rate. Try:
- 16000 Hz (Whisper standard)
- 22050 Hz (Piper TTS)
- 24000 Hz (Kokoro TTS)
- 44100 Hz (CD quality)
- 48000 Hz (professional)

### Issue: "ALSA lib ... underrun occurred"
**Solution**: Buffer underrun - increase buffer_size parameter

### Issue: Choppy/delayed audio
**Solution**: 
- Reduce chunk_size for lower latency
- Increase buffer_size for smoother playback
- Check system CPU load

### Issue: No audio heard in loopback test
**Solution**:
1. Check microphone works: `arecord -d 5 test.wav && aplay test.wav`
2. Check volume: `alsamixer`
3. Verify correct audio device selected
4. Check ROS topic is publishing: `ros2 topic hz /audio_stream`

## Expected Performance

- **Latency**: 
  - Loopback test: 100-300ms
  - Full pipeline: 500-2000ms (depends on Whisper/TTS)
- **CPU Usage**:
  - Audio streaming: <1%
  - Audio playback: <1%
  - Combined: <2%
- **Bandwidth**:
  - 16kHz mono int16: ~256 kbps
  - 24kHz mono int16: ~384 kbps

## Success Criteria

✅ **test_audio_streamer**: 
- Hear yourself with <500ms delay
- No dropouts or clicks
- Statistics show 0 dropped packets

✅ **test_audio_playback**:
- All 4 test sounds play clearly
- Correct pitch/tone
- No distortion

✅ **Full pipeline**:
- Speech recognized correctly
- TTS response heard clearly
- End-to-end latency <3 seconds
