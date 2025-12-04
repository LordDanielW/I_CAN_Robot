#!/usr/bin/env python3
"""
Whisper Server Node - Offline speech recognition using OpenAI Whisper
Subscribes to audio stream and publishes transcribed text
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import numpy as np
from faster_whisper import WhisperModel
import io
import wave

class WhisperServer(Node):
    def __init__(self):
        super().__init__('whisper_server')
        
        # Declare parameters
        self.declare_parameter('model_size', 'base.en')
        self.declare_parameter('device', 'cpu')  # 'cuda' or 'cpu'
        self.declare_parameter('compute_type', 'int8')  # 'int8', 'float16', 'float32'
        
        # Get parameters
        model_size = self.get_parameter('model_size').value
        device = self.get_parameter('device').value
        compute_type = self.get_parameter('compute_type').value
        
        # Subscribe to audio stream, Publish text
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'audio_stream',
            self.audio_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        
        # Initialize Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size} on {device}...')
        try:
            self.model = WhisperModel(
                model_size,
                device=device,
                compute_type=compute_type
            )
            self.get_logger().info('Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            raise
        
        # Audio buffer for accumulating stream
        self.audio_buffer = []
        self.sample_rate = 16000
        
        self.get_logger().info('Whisper Server ready. Waiting for audio stream...')
    
    def audio_callback(self, msg):
        """Process incoming audio stream data"""
        try:
            # Convert UInt8MultiArray to audio bytes
            audio_data = bytes(msg.data)
            
            # Accumulate audio data
            self.audio_buffer.append(audio_data)
            
            # Process when we have enough data (e.g., 3 seconds worth)
            # At 16kHz, 16-bit (2 bytes per sample), that's 16000 * 2 * 3 = 96000 bytes
            buffer_size = sum(len(chunk) for chunk in self.audio_buffer)
            
            if buffer_size >= 96000:  # ~3 seconds of audio
                self.process_audio()
                self.audio_buffer = []  # Clear buffer after processing
                
        except Exception as e:
            self.get_logger().error(f'Error in audio callback: {e}')
    
    def process_audio(self):
        """Transcribe accumulated audio buffer"""
        try:
            # Concatenate all buffered audio
            audio_bytes = b''.join(self.audio_buffer)
            
            # Convert bytes to numpy array (assuming 16-bit PCM)
            audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
            
            # Convert to float32 and normalize
            audio_float = audio_int16.astype(np.float32) / 32768.0
            
            # Transcribe using Whisper
            segments, info = self.model.transcribe(
                audio_float,
                language='en',
                beam_size=1,  # Faster inference
                vad_filter=True,  # Use built-in VAD
                vad_parameters=dict(
                    min_silence_duration_ms=500
                )
            )
            
            # Collect all segments
            transcription = ' '.join([segment.text.strip() for segment in segments])
            
            if transcription:
                self.get_logger().info(f'Recognized: {transcription}')
                
                # Publish transcribed text
                pub_msg = String()
                pub_msg.data = transcription
                self.publisher_.publish(pub_msg)
            
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')

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
