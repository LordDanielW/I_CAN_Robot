#!/bin/bash
set -e

echo "🔵 Downloading 'Ryan' (High Quality) Voice Model..."
mkdir -p ~/piper_voices
cd ~/piper_voices

# "Ryan" is widely considered one of the most natural sounding English voices for Piper
VOICE_NAME="en_US-ryan-high"

if [ ! -f "${VOICE_NAME}.onnx" ]; then
    echo "   -> Downloading ${VOICE_NAME}..."
    wget -q "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/ryan/high/${VOICE_NAME}.onnx"
    wget -q "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/ryan/high/${VOICE_NAME}.onnx.json"
else
    echo "   -> Voice model already exists."
fi

echo "🔵 Updating Python Script to use Ryan..."
# Overwrite the previous python script to point to the new model
cat > ~/compare_tts.py << 'EOF'
import pyttsx3
import os
import subprocess
import time

text = "Hello! I am the Unitree Go 2 robot. I have upgraded my voice module to high quality. How do I sound?"

print("\n--- TEST 1: pyttsx3 (Standard Robot) ---")
try:
    engine = pyttsx3.init()
    engine.setProperty('rate', 150)
    engine.say(text)
    engine.runAndWait()
except Exception as e:
    print(f"pyttsx3 error: {e}")

print("\n--- TEST 2: Piper TTS (Ryan - High Quality) ---")
# Pointing to the new Ryan model
voice_model = os.path.expanduser("~/piper_voices/en_US-ryan-high.onnx")

if not os.path.exists(voice_model):
    print(f"Error: Model not found at {voice_model}")
    exit(1)

# Using aplay for low-latency playback
cmd = f'echo "{text}" | piper --model {voice_model} --output-raw | aplay -r 22050 -f S16_LE -t raw - 2>/dev/null'

start_time = time.time()
try:
    subprocess.run(cmd, shell=True, check=True)
    end_time = time.time()
    print(f"\n[Latency Debug] Time to generate and speak: {end_time - start_time:.2f}s")
except Exception as e:
    print(f"Piper error: {e}")

print("\nDone! This should sound much more like a real person.")
EOF

echo "✅ Upgrade Complete!"
echo "👉 Run the comparison: python3 ~/compare_tts.py"