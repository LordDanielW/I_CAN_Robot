#!/bin/bash
set -e  # Exit on error

echo "🔵 [1/4] Installing System Audio Dependencies..."
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    portaudio19-dev \
    libespeak-ng1 \
    espeak-ng \
    ffmpeg \
    alsa-utils

echo "🔵 [2/4] Installing Python Libraries..."
# Note: On Ubuntu 24.04, we use --break-system-packages to install globally for user convenience
# ideally, you should use a venv, but for a dev robot, this is standard practice.
pip3 install --break-system-packages \
    pyttsx3 \
    piper-tts \
    sounddevice \
    numpy

echo "🔵 [3/4] Downloading a Neural Voice Model (Piper)..."
# We will download 'en_US-lessac-medium' (High quality, very fast)
mkdir -p ~/piper_voices
cd ~/piper_voices

VOICE_NAME="en_US-lessac-medium"
if [ ! -f "${VOICE_NAME}.onnx" ]; then
    echo "   -> Downloading ${VOICE_NAME}.onnx..."
    wget -q "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/medium/${VOICE_NAME}.onnx"
    wget -q "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/medium/${VOICE_NAME}.onnx.json"
else
    echo "   -> Voice model already exists."
fi

echo "🔵 [4/4] Creating Test Script (compare_tts.py)..."
cat > ~/compare_tts.py << 'EOF'
import pyttsx3
import os
import subprocess

text = "Hello! I am the Unitree Go 2 robot. This is a test of my audio systems."

print("\n--- TEST 1: pyttsx3 (The 'Robot' Voice) ---")
try:
    engine = pyttsx3.init()
    engine.setProperty('rate', 150)
    engine.say(text)
    engine.runAndWait()
except Exception as e:
    print(f"pyttsx3 error: {e}")

print("\n--- TEST 2: Piper TTS (The 'Human' Voice) ---")
voice_model = os.path.expanduser("~/piper_voices/en_US-lessac-medium.onnx")

# We use subprocess to pipe audio directly to aplay for lowest latency
cmd = f'echo "{text}" | piper --model {voice_model} --output-raw | aplay -r 22050 -f S16_LE -t raw - 2>/dev/null'

try:
    subprocess.run(cmd, shell=True, check=True)
except Exception as e:
    print(f"Piper error: {e}")
    print("Ensure 'aplay' is installed (sudo apt install alsa-utils)")

print("\nDone! Which one did you prefer?")
EOF

echo "✅ Setup Complete!"
echo "👉 Run the test now: python3 ~/compare_tts.py"