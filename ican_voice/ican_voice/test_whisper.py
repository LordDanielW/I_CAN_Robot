#!/usr/bin/env python3
"""
Whisper Test Script - GPU Accelerated
Tests faster-whisper with large-v3-turbo on CUDA
"""

import pyaudio
import numpy as np
import time
import sys

def test_whisper():
    print("=" * 60)
    print("Faster-Whisper GPU Test (large-v3-turbo)")
    print("=" * 60)
    
    # Configuration - Optimized for RTX 5070 Ti
    MODEL_SIZE = "large-v3-turbo"  # Best speed/accuracy balance
    DEVICE = "cuda"
    COMPUTE_TYPE = "float16"  # FP16 for RTX cards
    SAMPLE_RATE = 16000
    RECORD_SECONDS = 5
    BATCH_SIZE = 16
    
    try:
        from faster_whisper import WhisperModel, BatchedInferencePipeline
        import torch
        
        # Check CUDA
        if torch.cuda.is_available():
            gpu_name = torch.cuda.get_device_name(0)
            gpu_mem = torch.cuda.get_device_properties(0).total_memory / 1e9
            print(f"\n✓ CUDA available: {gpu_name} ({gpu_mem:.1f}GB)")
        else:
            print("\n✗ CUDA not available, falling back to CPU")
            DEVICE = "cpu"
            COMPUTE_TYPE = "int8"
        
        print(f"\nLoading model: {MODEL_SIZE} on {DEVICE} ({COMPUTE_TYPE})...")
        start_load = time.time()
        
        model = WhisperModel(MODEL_SIZE, device=DEVICE, compute_type=COMPUTE_TYPE)
        batched_model = BatchedInferencePipeline(model=model)
        
        load_time = time.time() - start_load
        print(f"✓ Model loaded in {load_time:.1f}s")
        
        # Audio setup
        p = pyaudio.PyAudio()
        
        print("\nAvailable audio input devices:")
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"  [{i}] {info['name']}")
        
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=1024
        )
        
        print(f"\n{'=' * 60}")
        print(f"🎤 RECORDING ({RECORD_SECONDS}s) - Speak now!")
        print(f"{'=' * 60}\n")
        
        # Record
        frames = []
        for _ in range(0, int(SAMPLE_RATE / 1024 * RECORD_SECONDS)):
            data = stream.read(1024, exception_on_overflow=False)
            frames.append(data)
        
        print("Recording complete. Transcribing...\n")
        
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        # Convert to float32
        audio_data = b''.join(frames)
        audio_int16 = np.frombuffer(audio_data, dtype=np.int16)
        audio_float = audio_int16.astype(np.float32) / 32768.0
        
        # Transcribe with batched inference
        start_time = time.time()
        
        segments, info = batched_model.transcribe(
            audio_float,
            language='en',
            batch_size=BATCH_SIZE,
            beam_size=1,
            vad_filter=True,
            vad_parameters=dict(min_silence_duration_ms=300)
        )
        
        transcription = ' '.join([seg.text.strip() for seg in segments])
        elapsed = time.time() - start_time
        
        # Calculate metrics
        audio_duration = len(audio_float) / SAMPLE_RATE
        rtf = elapsed / audio_duration if audio_duration > 0 else 0
        
        print("=" * 60)
        print("TRANSCRIPTION:")
        print("=" * 60)
        print(transcription if transcription else "(No speech detected)")
        print("=" * 60)
        print(f"\nMetrics:")
        print(f"  Audio duration: {audio_duration:.2f}s")
        print(f"  Processing time: {elapsed:.2f}s")
        print(f"  Real-Time Factor: {rtf:.3f}x {'✓ Real-time capable!' if rtf < 1 else ''}")
        print(f"  Language: {info.language} (prob: {info.language_probability:.2f})")
        
        print("\n✓ Whisper test successful!")
        return True
        
    except ImportError as e:
        print(f"\n✗ Import error: {e}")
        print("\nInstall with:")
        print("  pip install faster-whisper torch --index-url https://download.pytorch.org/whl/cu121")
        return False
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    success = test_whisper()
    sys.exit(0 if success else 1)