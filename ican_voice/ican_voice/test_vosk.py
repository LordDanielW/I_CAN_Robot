#!/usr/bin/env python3
"""
Simple Vosk Test Script (No ROS)
Tests Vosk speech recognition with microphone input
"""

import os
import json
import pyaudio
from vosk import Model, KaldiRecognizer

def test_vosk():
    print("Testing Vosk speech recognition...")
    
    # Check for model
    model_path = os.path.expanduser("~/vosk-model")
    
    if not os.path.exists(model_path):
        print(f"\n✗ Vosk model not found at {model_path}")
        print("\nDownload a model:")
        print("  cd ~")
        print("  wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip")
        print("  unzip vosk-model-small-en-us-0.15.zip")
        print("  mv vosk-model-small-en-us-0.15 vosk-model")
        return False
    
    try:
        print(f"Loading Vosk model from {model_path}...")
        model = Model(model_path)
        recognizer = KaldiRecognizer(model, 16000)
        
        print("Model loaded successfully!")
        
        # Setup audio
        p = pyaudio.PyAudio()
        
        # List audio devices
        print("\nAvailable audio input devices:")
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"  {i}: {info['name']} (channels: {info['maxInputChannels']})")
        
        print("\nOpening microphone stream...")
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=8000
        )
        stream.start_stream()
        
        print("\n" + "=" * 60)
        print("🎤 LISTENING... Speak now! (Press Ctrl+C to stop)")
        print("=" * 60 + "\n")
        
        try:
            while True:
                data = stream.read(4000, exception_on_overflow=False)
                
                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    text = result.get('text', '')
                    
                    if text:
                        print(f"Recognized: {text}")
                else:
                    # Partial result
                    partial = json.loads(recognizer.PartialResult())
                    partial_text = partial.get('partial', '')
                    if partial_text:
                        print(f"Partial: {partial_text}", end='\r')
        
        except KeyboardInterrupt:
            print("\n\nStopping...")
        
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()
        
        print("\n✓ Vosk test completed successfully!")
        return True
        
    except Exception as e:
        print(f"\n✗ Vosk test failed: {e}")
        print("\nTroubleshooting:")
        print("  - Install vosk: pip install vosk")
        print("  - Install pyaudio: sudo apt install python3-pyaudio")
        print("  - Check microphone: arecord -l")
        return False

if __name__ == '__main__':
    print("=" * 60)
    print("Vosk Speech Recognition Test Script")
    print("=" * 60 + "\n")
    test_vosk()
