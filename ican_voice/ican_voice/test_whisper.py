#!/usr/bin/env python3
"""
Simple Whisper Test Script (No ROS)
Tests faster-whisper offline speech recognition with microphone
"""

import pyaudio
import numpy as np
from faster_whisper import WhisperModel
import time

def test_whisper():
    print("Testing faster-whisper (OpenAI Whisper offline)...")
    
    # Configuration
    MODEL_SIZE = "base.en"  # tiny.en, base.en, small.en, medium.en, large-v2
    DEVICE = "cpu"  # "cuda" or "cpu"
    COMPUTE_TYPE = "int8"  # "int8", "float16", "float32"
    SAMPLE_RATE = 16000
    RECORD_SECONDS = 5
    
    try:
        print(f"\nLoading Whisper model: {MODEL_SIZE} on {DEVICE}...")
        model = WhisperModel(MODEL_SIZE, device=DEVICE, compute_type=COMPUTE_TYPE)
        print("Model loaded successfully!")
        
        # Setup audio
        p = pyaudio.PyAudio()
        
        # List audio devices
        print("\nAvailable audio input devices:")
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"  {i}: {info['name']} (channels: {info['maxInputChannels']})")
        
        print(f"\nOpening microphone stream (recording for {RECORD_SECONDS} seconds)...")
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=1024
        )
        
        print("\n" + "=" * 60)
        print(f"🎤 RECORDING... Speak now! ({RECORD_SECONDS} seconds)")
        print("=" * 60 + "\n")
        
        # Record audio
        frames = []
        for _ in range(0, int(SAMPLE_RATE / 1024 * RECORD_SECONDS)):
            data = stream.read(1024, exception_on_overflow=False)
            frames.append(data)
        
        print("Recording finished. Processing...\n")
        
        # Stop and close stream
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        # Convert to numpy array
        audio_data = b''.join(frames)
        audio_int16 = np.frombuffer(audio_data, dtype=np.int16)
        audio_float = audio_int16.astype(np.float32) / 32768.0
        
        # Transcribe
        print("Transcribing with Whisper...")
        start_time = time.time()
        
        segments, info = model.transcribe(
            audio_float,
            language='en',
            beam_size=1,
            vad_filter=True,
            vad_parameters=dict(min_silence_duration_ms=500)
        )
        
        # Collect results
        transcription = ' '.join([segment.text.strip() for segment in segments])
        elapsed_time = time.time() - start_time
        
        print("\n" + "=" * 60)
        print("TRANSCRIPTION:")
        print("=" * 60)
        if transcription:
            print(transcription)
        else:
            print("(No speech detected)")
        print("=" * 60)
        print(f"\nProcessing time: {elapsed_time:.2f} seconds")
        print(f"Detected language: {info.language} (probability: {info.language_probability:.2f})")
        
        print("\n✓ Whisper test successful!")
        return True
        
    except Exception as e:
        print(f"\n✗ Whisper test failed: {e}")
        print("\nTroubleshooting:")
        print("  - Install faster-whisper: pip install faster-whisper")
        print("  - Install pyaudio: sudo apt install python3-pyaudio")
        print("  - Check microphone: arecord -l")
        return False

if __name__ == '__main__':
    print("=" * 60)
    print("OpenAI Whisper (faster-whisper) Test Script")
    print("=" * 60 + "\n")
    test_whisper()
