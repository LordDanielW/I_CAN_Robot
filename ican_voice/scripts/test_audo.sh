#!/bin/bash
# #################################
# ROS2 Audio Streaming Diagnostic Script
# For Ubuntu 22.04 with Bluetooth Audio (Airhug1)
# Diagnoses why ros2 audio nodes can't be heard
# #################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "========================================"
echo "🔊 ROS2 Audio Streaming Diagnostics"
echo "========================================"
echo ""

# Track issues found
ISSUES=()

# #################################
# 1. Check Audio Subsystems
# #################################

echo -e "${BLUE}[1/10] Checking Audio Subsystems...${NC}"

echo -n "  PulseAudio: "
if pulseaudio --check 2>/dev/null; then
    echo -e "${GREEN}Running${NC}"
    PA_RUNNING=true
else
    echo -e "${YELLOW}Not running${NC}"
    PA_RUNNING=false
    ISSUES+=("PulseAudio not running")
fi

echo -n "  JACK: "
if pgrep -x jackd > /dev/null || pgrep -x jackdbus > /dev/null; then
    echo -e "${YELLOW}Running (may conflict with PulseAudio)${NC}"
    JACK_RUNNING=true
    ISSUES+=("JACK is running - may intercept audio from PulseAudio")
else
    echo -e "${GREEN}Not running${NC}"
    JACK_RUNNING=false
fi

echo -n "  PipeWire: "
if pgrep -x pipewire > /dev/null; then
    echo -e "${YELLOW}Running${NC}"
    PIPEWIRE_RUNNING=true
else
    echo -e "${GREEN}Not running${NC}"
    PIPEWIRE_RUNNING=false
fi

# #################################
# 2. Bluetooth Status
# #################################

echo ""
echo -e "${BLUE}[2/10] Checking Bluetooth...${NC}"

echo -n "  Bluetooth service: "
if systemctl is-active --quiet bluetooth; then
    echo -e "${GREEN}Active${NC}"
else
    echo -e "${RED}Inactive${NC}"
    ISSUES+=("Bluetooth service not running")
fi

echo "  Connected devices:"
bluetoothctl devices Connected 2>/dev/null | while read line; do
    echo "    $line"
done

echo "  Paired devices:"
bluetoothctl devices Paired 2>/dev/null | head -5 | while read line; do
    echo "    $line"
done

# #################################
# 3. PulseAudio Sinks (Output Devices)
# #################################

echo ""
echo -e "${BLUE}[3/10] PulseAudio Output Devices (Sinks)...${NC}"

if $PA_RUNNING; then
    echo "  Available sinks:"
    pactl list sinks short 2>/dev/null | while read line; do
        echo "    $line"
    done
    
    echo ""
    echo "  Default sink:"
    DEFAULT_SINK=$(pactl get-default-sink 2>/dev/null || echo "unknown")
    echo "    $DEFAULT_SINK"
    
    # Check if Bluetooth device is the default
    if [[ "$DEFAULT_SINK" == *"bluez"* ]] || [[ "$DEFAULT_SINK" == *"bluetooth"* ]]; then
        echo -e "    ${GREEN}✓ Bluetooth device is default sink${NC}"
    else
        echo -e "    ${YELLOW}⚠ Bluetooth may not be default sink${NC}"
        ISSUES+=("Bluetooth device may not be default PulseAudio sink")
    fi
fi

# #################################
# 4. PulseAudio Sources (Input Devices)
# #################################

echo ""
echo -e "${BLUE}[4/10] PulseAudio Input Devices (Sources)...${NC}"

if $PA_RUNNING; then
    echo "  Available sources:"
    pactl list sources short 2>/dev/null | while read line; do
        echo "    $line"
    done
    
    echo ""
    echo "  Default source:"
    DEFAULT_SOURCE=$(pactl get-default-source 2>/dev/null || echo "unknown")
    echo "    $DEFAULT_SOURCE"
fi

# #################################
# 5. Bluetooth Audio Profile
# #################################

echo ""
echo -e "${BLUE}[5/10] Bluetooth Audio Profile...${NC}"

if $PA_RUNNING; then
    echo "  Bluetooth card profiles:"
    pactl list cards 2>/dev/null | grep -A 30 "bluez" | grep -E "(Name:|Active Profile:|Profiles:)" | head -10 | while read line; do
        echo "    $line"
    done
    
    # Check for A2DP vs HSP
    ACTIVE_PROFILE=$(pactl list cards 2>/dev/null | grep -A 30 "bluez" | grep "Active Profile:" | head -1)
    if [[ "$ACTIVE_PROFILE" == *"a2dp"* ]]; then
        echo -e "    ${GREEN}✓ A2DP profile (high quality stereo)${NC}"
    elif [[ "$ACTIVE_PROFILE" == *"hsp"* ]] || [[ "$ACTIVE_PROFILE" == *"hfp"* ]]; then
        echo -e "    ${YELLOW}⚠ HSP/HFP profile (mono, low quality)${NC}"
        ISSUES+=("Bluetooth using HSP/HFP profile - switch to A2DP for better quality")
    else
        echo -e "    ${YELLOW}Could not determine profile${NC}"
    fi
fi

# #################################
# 6. ALSA Devices
# #################################

echo ""
echo -e "${BLUE}[6/10] ALSA Devices...${NC}"

echo "  Playback devices (aplay -l):"
aplay -l 2>/dev/null | grep -E "^card|^  Subdevice" | head -10 | while read line; do
    echo "    $line"
done

echo ""
echo "  Recording devices (arecord -l):"
arecord -l 2>/dev/null | grep -E "^card|^  Subdevice" | head -10 | while read line; do
    echo "    $line"
done

# #################################
# 7. PyAudio Devices (What ROS2 sees)
# #################################

echo ""
echo -e "${BLUE}[7/10] PyAudio Devices (What Python/ROS2 sees)...${NC}"

python3 << 'EOF'
try:
    import pyaudio
    p = pyaudio.PyAudio()
    
    print("  Output devices:")
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info['maxOutputChannels'] > 0:
            default = " [DEFAULT]" if i == p.get_default_output_device_info()['index'] else ""
            print(f"    [{i}] {info['name']} (out:{info['maxOutputChannels']}, rate:{int(info['defaultSampleRate'])}){default}")
    
    print("\n  Input devices:")
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info['maxInputChannels'] > 0:
            default = " [DEFAULT]" if i == p.get_default_input_device_info()['index'] else ""
            print(f"    [{i}] {info['name']} (in:{info['maxInputChannels']}, rate:{int(info['defaultSampleRate'])}){default}")
    
    p.terminate()
except Exception as e:
    print(f"  ERROR: {e}")
    print("  Install pyaudio: sudo apt install python3-pyaudio")
EOF

# #################################
# 8. Sample Rate Check
# #################################

echo ""
echo -e "${BLUE}[8/10] Sample Rate Analysis...${NC}"

echo "  Common TTS sample rates:"
echo "    Kokoro TTS:  24000 Hz"
echo "    Piper TTS:   22050 Hz"
echo "    Whisper:     16000 Hz"

echo ""
echo "  Device supported rates:"
python3 << 'EOF'
try:
    import pyaudio
    p = pyaudio.PyAudio()
    
    default_output = p.get_default_output_device_info()
    device_idx = default_output['index']
    device_name = default_output['name']
    
    print(f"  Testing default output: {device_name}")
    
    test_rates = [16000, 22050, 24000, 44100, 48000]
    for rate in test_rates:
        try:
            supported = p.is_format_supported(
                rate,
                output_device=device_idx,
                output_channels=1,
                output_format=pyaudio.paInt16
            )
            print(f"    {rate} Hz: ✓ Supported")
        except:
            print(f"    {rate} Hz: ✗ NOT supported")
    
    p.terminate()
except Exception as e:
    print(f"  ERROR: {e}")
EOF

# #################################
# 9. Test Audio Generation
# #################################

echo ""
echo -e "${BLUE}[9/10] Audio Playback Tests...${NC}"

# Generate test tones at different sample rates
echo "  Generating test tones..."

python3 << 'EOF'
import numpy as np
import struct
import os

def generate_tone(filename, sample_rate, duration=1.0, freq=440):
    """Generate a sine wave tone"""
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    tone = np.sin(2 * np.pi * freq * t) * 0.5
    audio_int16 = (tone * 32767).astype(np.int16)
    
    # Write raw PCM
    with open(filename, 'wb') as f:
        f.write(audio_int16.tobytes())
    return len(audio_int16)

# Generate tones at ROS2-relevant sample rates
rates = [16000, 22050, 24000]
for rate in rates:
    filename = f"/tmp/test_tone_{rate}.raw"
    samples = generate_tone(filename, rate, duration=0.5, freq=440)
    print(f"  Generated: {filename} ({rate}Hz, {samples} samples)")

print("\n  To test playback manually:")
print("    aplay -r 24000 -f S16_LE -c 1 /tmp/test_tone_24000.raw")
print("    aplay -r 22050 -f S16_LE -c 1 /tmp/test_tone_22050.raw")
print("    aplay -r 16000 -f S16_LE -c 1 /tmp/test_tone_16000.raw")
EOF

echo ""
echo "  Testing PulseAudio playback (24kHz)..."
if command -v paplay &> /dev/null; then
    # Convert raw to wav for paplay
    python3 << 'EOF'
import wave
import numpy as np

# Generate a short beep
sample_rate = 24000
duration = 0.3
freq = 880

t = np.linspace(0, duration, int(sample_rate * duration), False)
tone = np.sin(2 * np.pi * freq * t) * 0.3
audio_int16 = (tone * 32767).astype(np.int16)

with wave.open('/tmp/test_beep.wav', 'w') as wav:
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(sample_rate)
    wav.writeframes(audio_int16.tobytes())

print("    Playing test beep via PulseAudio...")
EOF
    paplay /tmp/test_beep.wav 2>/dev/null && echo -e "    ${GREEN}✓ PulseAudio playback works${NC}" || echo -e "    ${RED}✗ PulseAudio playback failed${NC}"
else
    echo "    paplay not available"
fi

# #################################
# 10. ROS2 Audio Topics
# #################################

echo ""
echo -e "${BLUE}[10/10] ROS2 Audio Topics...${NC}"

if command -v ros2 &> /dev/null; then
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    
    echo "  Active audio-related topics:"
    ros2 topic list 2>/dev/null | grep -E "(audio|tts|speech)" | while read topic; do
        hz=$(timeout 2 ros2 topic hz "$topic" 2>/dev/null | head -1 || echo "no data")
        echo "    $topic - $hz"
    done
    
    if [ -z "$(ros2 topic list 2>/dev/null | grep -E '(audio|tts)')" ]; then
        echo "    No audio topics found (is a node running?)"
    fi
else
    echo "  ROS2 not sourced"
fi

# #################################
# Summary & Recommendations
# #################################

echo ""
echo "========================================"
echo -e "${BLUE}📋 SUMMARY${NC}"
echo "========================================"

if [ ${#ISSUES[@]} -eq 0 ]; then
    echo -e "${GREEN}No obvious issues detected!${NC}"
else
    echo -e "${YELLOW}Issues found:${NC}"
    for issue in "${ISSUES[@]}"; do
        echo -e "  ${RED}•${NC} $issue"
    done
fi

echo ""
echo "========================================"
echo -e "${BLUE}🔧 COMMON FIXES${NC}"
echo "========================================"

cat << 'FIXES'

1. WRONG SAMPLE RATE (Most Common!)
   Your ROS2 node outputs 24kHz (Kokoro) or 22050Hz (Piper),
   but device may expect different rate.
   
   Fix in audio_playback_node.py:
   ```python
   self.stream = self.p.open(
       format=pyaudio.paInt16,
       channels=1,
       rate=24000,  # Match your TTS output!
       output=True
   )
   ```

2. PYAUDIO USING WRONG DEVICE
   PyAudio may not use Bluetooth as default.
   
   Fix - specify device index:
   ```python
   # Find your device index from the diagnostic output above
   self.stream = self.p.open(
       format=pyaudio.paInt16,
       channels=1,
       rate=24000,
       output=True,
       output_device_index=5  # Your Bluetooth device index
   )
   ```

3. JACK INTERCEPTING AUDIO
   If JACK is running, it may grab audio before PulseAudio.
   
   Fix:
   ```bash
   # Stop JACK
   killall jackd jackdbus 2>/dev/null
   
   # Restart PulseAudio
   pulseaudio -k && pulseaudio --start
   ```

4. BLUETOOTH PROFILE WRONG
   HSP/HFP is for calls (mono, bad quality).
   A2DP is for music (stereo, good quality).
   
   Fix:
   ```bash
   # List Bluetooth card
   pactl list cards | grep -E "(Name:|bluez)"
   
   # Switch to A2DP (replace CARD_NAME)
   pactl set-card-profile bluez_card.XX_XX_XX_XX_XX_XX a2dp-sink
   ```

5. SET DEFAULT SINK TO BLUETOOTH
   ```bash
   # List sinks
   pactl list sinks short
   
   # Set Bluetooth as default (replace SINK_NAME)
   pactl set-default-sink bluez_sink.XX_XX_XX_XX_XX_XX.a2dp_sink
   ```

6. TEST PYAUDIO DIRECTLY
   ```python
   import pyaudio
   import numpy as np
   
   p = pyaudio.PyAudio()
   stream = p.open(format=pyaudio.paInt16, channels=1, rate=24000, output=True)
   
   # Generate beep
   t = np.linspace(0, 0.5, 12000, False)
   tone = (np.sin(2 * np.pi * 440 * t) * 16000).astype(np.int16)
   stream.write(tone.tobytes())
   
   stream.close()
   p.terminate()
   ```

7. USE PULSEAUDIO DIRECTLY (bypass PyAudio issues)
   ```python
   import subprocess
   
   def play_audio(audio_bytes, sample_rate=24000):
       cmd = ['paplay', '--raw', f'--rate={sample_rate}', 
              '--format=s16le', '--channels=1']
       proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
       proc.communicate(input=audio_bytes)
   ```

FIXES

echo ""
echo "========================================"
echo "Run specific tests:"
echo "  aplay -r 24000 -f S16_LE -c 1 /tmp/test_tone_24000.raw"
echo "  paplay /tmp/test_beep.wav"
echo "========================================"