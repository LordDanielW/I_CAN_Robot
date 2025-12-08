#!/usr/bin/env python3
"""
Whisper Server Node - GPU-Accelerated Speech Recognition
Uses faster-whisper with large-v3-turbo for RTX 5070 Ti
Subscribes to audio stream and publishes transcribed text
"""

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
        try:
            self.model = WhisperModel(
                model_size,
                device=device,
                compute_type=compute_type,
                download_root=None,  # Use default cache
            )
            
            # Enable batched inference for GPU efficiency
            self.batched_model = BatchedInferencePipeline(model=self.model)
            self.get_logger().info('Whisper model loaded with batched inference pipeline')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            self.get_logger().warn('Falling back to CPU mode...')
            self.model = WhisperModel(model_size, device='cpu', compute_type='int8')
            self.batched_model = None
        
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
        self.get_logger().info(f'  Model: {model_size}, Device: {device}, Compute: {compute_type}')
        self.get_logger().info(f'  Buffer: {self.buffer_seconds}s, Batch size: {self.batch_size}')
    
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