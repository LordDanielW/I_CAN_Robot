#!/usr/bin/env python3
"""
Whisper Server Node - GPU-Accelerated Speech Recognition
Uses faster-whisper with large-v3-turbo for RTX 5070 Ti
Subscribes to audio stream and publishes transcribed text
"""

import os
import sys

# Fix cuDNN library path for pip-installed nvidia-cudnn-cu12
def setup_cudnn_path():
    """Add pip-installed cuDNN to library path"""
    try:
        import site
        user_site = site.getusersitepackages()
        
        # Common locations for pip-installed CUDA libraries
        cuda_paths = [
            os.path.join(user_site, 'nvidia', 'cudnn', 'lib'),
            os.path.join(user_site, 'nvidia/cudnn/lib'),
            os.path.expanduser('~/.local/lib/python3.10/site-packages/nvidia/cudnn/lib'),
            '/usr/local/lib/python3.10/dist-packages/nvidia/cudnn/lib',
        ]
        
        existing_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
        new_paths = [p for p in cuda_paths if os.path.exists(p)]
        
        if new_paths:
            os.environ['LD_LIBRARY_PATH'] = ':'.join(new_paths + [existing_ld_path])
            return True
        return False
    except Exception:
        return False

# Setup cuDNN path before importing torch/faster-whisper
setup_cudnn_path()

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import numpy as np
from faster_whisper import WhisperModel, BatchedInferencePipeline
import threading
from collections import deque
import time

class WhisperServer(Node):
    def __init__(self):
        super().__init__('whisper_server')
        
        # Declare parameters - optimized for RTX 5070 Ti (16GB VRAM)
        self.declare_parameter('model_size', 'large-v3-turbo')  # Best speed/accuracy
        self.declare_parameter('device', 'cuda')  # GPU acceleration
        self.declare_parameter('compute_type', 'float16')  # FP16 for RTX cards
        self.declare_parameter('batch_size', 16)  # Batched inference
        self.declare_parameter('beam_size', 1)  # Fast inference
        self.declare_parameter('buffer_seconds', 2.0)  # Audio buffer size
        self.declare_parameter('language', 'en')  # Target language
        
        # Get parameters
        model_size = self.get_parameter('model_size').value
        device = self.get_parameter('device').value
        compute_type = self.get_parameter('compute_type').value
        self.batch_size = self.get_parameter('batch_size').value
        self.beam_size = self.get_parameter('beam_size').value
        self.buffer_seconds = self.get_parameter('buffer_seconds').value
        self.language = self.get_parameter('language').value
        
        # Subscribe to audio stream
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'audio_stream',
            self.audio_callback,
            10
        )
        
        # Publishers
        self.text_pub = self.create_publisher(String, 'speech_text', 10)
        self.partial_pub = self.create_publisher(String, 'speech_partial', 10)
        
        # Initialize Whisper model with GPU optimization
        self.get_logger().info(f'Loading Whisper model: {model_size} on {device} ({compute_type})...')
        
        self.model = None
        self.batched_model = None
        
        # Try GPU first, fall back to CPU if needed
        if device == 'cuda':
            try:
                import torch
                if not torch.cuda.is_available():
                    raise RuntimeError("CUDA not available")
                
                self.get_logger().info(f'CUDA available: {torch.cuda.get_device_name(0)}')
                
                self.model = WhisperModel(
                    model_size,
                    device=device,
                    compute_type=compute_type,
                    download_root=None,
                )
                
                # Test the model with a small array to catch cuDNN errors early
                test_audio = np.zeros(16000, dtype=np.float32)
                try:
                    list(self.model.transcribe(test_audio, language=self.language))
                    self.get_logger().info('✓ GPU inference test passed')
                    
                    # Enable batched inference for GPU efficiency
                    self.batched_model = BatchedInferencePipeline(model=self.model)
                    self.get_logger().info('✓ Whisper model loaded with batched inference pipeline')
                    
                except Exception as e:
                    self.get_logger().warn(f'GPU inference test failed: {e}')
                    raise RuntimeError("GPU inference not working")
                
            except Exception as e:
                self.get_logger().warn(f'Failed to load GPU model: {e}')
                self.get_logger().info('Falling back to CPU mode...')
                device = 'cpu'
                compute_type = 'int8'
                self.model = None
        
        # CPU fallback
        if self.model is None:
            try:
                self.get_logger().info(f'Loading Whisper on CPU ({compute_type})...')
                self.model = WhisperModel(
                    model_size,
                    device='cpu',
                    compute_type=compute_type,
                    download_root=None,
                )
                self.get_logger().info('✓ Whisper model loaded on CPU')
            except Exception as e:
                self.get_logger().error(f'Failed to load Whisper model: {e}')
                raise
        
        # Audio buffer configuration
        self.sample_rate = 16000
        self.bytes_per_sample = 2  # 16-bit audio
        self.buffer_size = int(self.sample_rate * self.bytes_per_sample * self.buffer_seconds)
        
        # Thread-safe audio buffer
        self.audio_buffer = deque(maxlen=self.buffer_size)
        self.buffer_lock = threading.Lock()
        
        # Processing state
        self.is_processing = False
        self.last_transcription = ""
        
        # Timer for periodic processing
        self.process_timer = self.create_timer(0.5, self.check_and_process)
        
        self.get_logger().info(f'Whisper Server ready. Listening on /audio_stream...')
        self.get_logger().info(f'  Model: {model_size}')
        self.get_logger().info(f'  Device: {"GPU (batched)" if self.batched_model else "GPU" if device == "cuda" else "CPU"}')
        self.get_logger().info(f'  Compute: {compute_type}')
        self.get_logger().info(f'  Buffer: {self.buffer_seconds}s, Batch size: {self.batch_size}')
        
        if device == 'cpu':
            self.get_logger().warn('⚠️  Running on CPU - speech recognition will be slower')
            self.get_logger().info('To fix GPU issues:')
            self.get_logger().info('  1. Install cuDNN: sudo apt install libcudnn8 libcudnn8-dev')
            self.get_logger().info('  2. Or use CPU: --ros-args -p device:=cpu')
    
    def audio_callback(self, msg):
        """Accumulate incoming audio data"""
        with self.buffer_lock:
            self.audio_buffer.extend(msg.data)
    
    def check_and_process(self):
        """Check buffer and process when ready"""
        with self.buffer_lock:
            buffer_size = len(self.audio_buffer)
        
        # Process when we have enough data
        if buffer_size >= self.buffer_size and not self.is_processing:
            self.is_processing = True
            # Run in separate thread to not block ROS
            threading.Thread(target=self.process_audio, daemon=True).start()
    
    def process_audio(self):
        """Transcribe accumulated audio buffer"""
        try:
            # Get audio data from buffer
            with self.buffer_lock:
                audio_bytes = bytes(list(self.audio_buffer))
                self.audio_buffer.clear()
            
            if len(audio_bytes) < 1000:  # Skip if too short
                self.is_processing = False
                return
            
            # Convert bytes to numpy array (16-bit PCM)
            audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
            
            # Convert to float32 and normalize for Whisper
            audio_float = audio_int16.astype(np.float32) / 32768.0
            
            # Transcribe using batched inference if available
            start_time = time.time()
            
            if self.batched_model:
                segments, info = self.batched_model.transcribe(
                    audio_float,
                    language=self.language,
                    batch_size=self.batch_size,
                    beam_size=self.beam_size,
                    vad_filter=True,
                    vad_parameters=dict(
                        min_silence_duration_ms=300,
                        speech_pad_ms=100,
                    ),
                    without_timestamps=True,  # Faster
                )
            else:
                segments, info = self.model.transcribe(
                    audio_float,
                    language=self.language,
                    beam_size=self.beam_size,
                    vad_filter=True,
                    vad_parameters=dict(
                        min_silence_duration_ms=300,
                        speech_pad_ms=100,
                    ),
                )
            
            # Collect transcription
            transcription = ' '.join([seg.text.strip() for seg in segments])
            elapsed = time.time() - start_time
            
            if transcription and transcription != self.last_transcription:
                self.last_transcription = transcription
                
                self.get_logger().info(f'[{elapsed:.2f}s] Recognized: {transcription}')
                
                # Publish final transcription
                msg = String()
                msg.data = transcription
                self.text_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')
        
        finally:
            self.is_processing = False

def main(args=None):
    rclpy.init(args=args)
    node = WhisperServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()