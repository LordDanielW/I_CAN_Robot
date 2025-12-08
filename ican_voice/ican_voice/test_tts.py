#!/usr/bin/env python3
"""
TTS Test Script - Kokoro vs Piper Comparison
Tests GPU-accelerated Kokoro-82M and Piper TTS
WSL-compatible: Saves audio files instead of direct playback
"""

import time
import sys
import os
import wave

# Check if running on WSL
IS_WSL = 'microsoft' in os.uname().release.lower()

def save_wav(filename, audio, sample_rate):
    """Save audio as WAV file"""
    import numpy as np
    
    # Convert float to int16
    if audio.dtype == np.float32 or audio.dtype == np.float64:
        audio = (audio * 32767).astype(np.int16)
    
    with wave.open(filename, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)  # 16-bit
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(audio.tobytes())
    
    print(f"💾 Saved: {filename}")

def play_or_save_audio(audio, sample_rate, filename):
    """Play audio if possible, otherwise save to file"""
    if IS_WSL:
        print(f"\n💾 WSL detected - saving audio to file...")
        save_wav(filename, audio, sample_rate)
        print(f"   Play on Windows: start {filename}")
        return True
    else:
        try:
            import sounddevice as sd
            print("\n🔊 Playing audio...")
            sd.play(audio, samplerate=sample_rate)
            sd.wait()
            
            # Also save file
            save_wav(filename, audio, sample_rate)
            return True
        except Exception as e:
            print(f"✗ Playback error: {e}")
            print(f"   Saving to file instead...")
            save_wav(filename, audio, sample_rate)
            return False

def test_kokoro():
    """Test Kokoro TTS"""
    print("\n" + "=" * 60)
    print("TEST 1: Kokoro-82M (GPU)")
    print("=" * 60)
    
    try:
        import torch
        from kokoro import KPipeline
        import numpy as np
        
        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Device: {device}")
        
        if device == "cuda":
            print(f"GPU: {torch.cuda.get_device_name(0)}")
        
        # Initialize pipeline
        print("Loading Kokoro pipeline...")
        start = time.time()
        pipeline = KPipeline(lang_code='a')  # American English
        load_time = time.time() - start
        print(f"✓ Loaded in {load_time:.2f}s")
        
        # Test text
        text = "Hello! I am a Unitree Go 2 robot using Kokoro text to speech. This sounds much more natural than traditional robot voices."
        voice = "af_heart"  # American Female "Heart" voice
        
        print(f"\nGenerating speech for: \"{text[:50]}...\"")
        print(f"Voice: {voice}")
        
        # Generate audio
        start = time.time()
        audio_chunks = []
        
        for gs, ps, audio in pipeline(text, voice=voice, speed=1.0):
            if audio is not None:
                audio_chunks.append(audio)
        
        if audio_chunks:
            audio = np.concatenate(audio_chunks)
            gen_time = time.time() - start
            duration = len(audio) / 24000
            rtf = gen_time / duration
            
            print(f"\n✓ Generated {len(audio)} samples in {gen_time:.2f}s")
            print(f"  Audio duration: {duration:.2f}s")
            print(f"  Real-Time Factor: {rtf:.3f}x {'✓ Real-time!' if rtf < 1 else ''}")
            
            # Play or save audio
            play_or_save_audio(audio, 24000, "kokoro_test.wav")
            
            return True, gen_time, duration
        else:
            print("✗ No audio generated")
            return False, 0, 0
            
    except ImportError as e:
        print(f"✗ Kokoro not installed: {e}")
        print("\nInstall with:")
        print("  pip install kokoro soundfile sounddevice")
        print("  apt install espeak-ng")
        return None, 0, 0
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False, 0, 0


def test_piper():
    """Test Piper TTS"""
    print("\n" + "=" * 60)
    print("TEST 2: Piper TTS (CPU/ONNX)")
    print("=" * 60)
    
    try:
        import subprocess
        import numpy as np
        
        # Find Piper model
        model_paths = [
            os.path.expanduser("~/piper_voices/en_US-ryan-high.onnx"),
            os.path.expanduser("~/piper_voices/en_US-lessac-medium.onnx"),
            "/usr/share/piper-voices/en_US-lessac-medium.onnx",
        ]
        
        model_path = None
        for path in model_paths:
            if os.path.exists(path):
                model_path = path
                break
        
        if not model_path:
            print("✗ No Piper model found")
            print("  Download from: https://huggingface.co/rhasspy/piper-voices")
            return None, 0, 0
        
        print(f"Model: {os.path.basename(model_path)}")
        
        text = "Hello! I am a Unitree Go 2 robot using Piper text to speech. Compare my voice quality to Kokoro."
        
        print(f"\nGenerating speech for: \"{text[:50]}...\"")
        
        # Generate with Piper
        cmd = ['piper', '--model', model_path, '--output-raw']
        
        start = time.time()
        process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        audio_bytes, stderr = process.communicate(input=text.encode('utf-8'))
        gen_time = time.time() - start
        
        if process.returncode != 0:
            print(f"✗ Piper error: {stderr.decode()}")
            return False, 0, 0
        
        # Convert to numpy
        audio = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32) / 32768.0
        
        # Determine sample rate from model name
        sample_rate = 22050 if 'high' in model_path else 22050
        duration = len(audio) / sample_rate
        rtf = gen_time / duration
        
        print(f"\n✓ Generated {len(audio_bytes)} bytes in {gen_time:.2f}s")
        print(f"  Audio duration: {duration:.2f}s")
        print(f"  Real-Time Factor: {rtf:.3f}x {'✓ Real-time!' if rtf < 1 else ''}")
        
        # Play or save audio
        play_or_save_audio(audio, sample_rate, "piper_test.wav")
        
        return True, gen_time, duration
        
    except FileNotFoundError:
        print("✗ Piper not installed")
        print("\nInstall with:")
        print("  pip install piper-tts")
        return None, 0, 0
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False, 0, 0


def main():
    print("=" * 60)
    print("TTS Comparison Test: Kokoro vs Piper")
    print("=" * 60)
    
    if IS_WSL:
        print("🪟 WSL detected - audio will be saved to WAV files")
        print("   Play files with: explorer.exe <filename>.wav")
        print()
    
    results = {}
    
    # Test Kokoro
    kokoro_result, kokoro_time, kokoro_dur = test_kokoro()
    if kokoro_result is not None:
        results['kokoro'] = {
            'success': kokoro_result,
            'gen_time': kokoro_time,
            'duration': kokoro_dur,
            'rtf': kokoro_time / kokoro_dur if kokoro_dur > 0 else 0
        }
    
    time.sleep(1)
    
    # Test Piper
    piper_result, piper_time, piper_dur = test_piper()
    if piper_result is not None:
        results['piper'] = {
            'success': piper_result,
            'gen_time': piper_time,
            'duration': piper_dur,
            'rtf': piper_time / piper_dur if piper_dur > 0 else 0
        }
    
    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    if 'kokoro' in results and 'piper' in results:
        print(f"\n{'Engine':<12} {'Gen Time':<12} {'RTF':<10} {'Quality'}")
        print("-" * 50)
        print(f"{'Kokoro':<12} {results['kokoro']['gen_time']:.2f}s{'':<7} {results['kokoro']['rtf']:.3f}x{'':<5} Higher")
        print(f"{'Piper':<12} {results['piper']['gen_time']:.2f}s{'':<7} {results['piper']['rtf']:.3f}x{'':<5} Good")
        
        if results['kokoro']['rtf'] < results['piper']['rtf']:
            speedup = results['piper']['rtf'] / results['kokoro']['rtf']
            print(f"\n✓ Kokoro is {speedup:.1f}x faster than Piper")
        
    print("\nRecommendation for RTX 5070 Ti:")
    print("  → Use Kokoro-82M for best quality and GPU acceleration")
    print("  → Keep Piper as CPU fallback")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())