#!/usr/bin/env python3
"""
Test FFmpeg Audio Streaming - Sends kokoro_test.wav via ROS topic
Run this alongside audio_playback_ffmpeg.py to test streaming
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import os
import wave
import time


class TestFFmpegStreamer(Node):
    def __init__(self):
        super().__init__('test_ffmpeg_streamer')
        
        # Parameters
        self.declare_parameter('chunk_size', 4096)
        self.declare_parameter('stream_delay', 0.01)  # Delay between chunks
        
        self.chunk_size = self.get_parameter('chunk_size').value
        self.stream_delay = self.get_parameter('stream_delay').value
        
        # Path to kokoro_test.wav
        self.wav_file = os.path.join(
            os.path.dirname(__file__), 
            'kokoro_test.wav'
        )
        
        # Publishers
        self.audio_pub = self.create_publisher(
            UInt8MultiArray,
            'audio/tts_stream',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            'tts/status',
            10
        )
        
        self.get_logger().info('🎵 FFmpeg Streaming Test Node')
        self.get_logger().info(f'   WAV file: {self.wav_file}')
        self.get_logger().info(f'   Chunk size: {self.chunk_size} bytes')
        
        if not os.path.exists(self.wav_file):
            self.get_logger().error(f'WAV file not found: {self.wav_file}')
            return
        
        # Stream the file after a short delay
        self.create_timer(2.0, self.stream_wav_file)
    
    def stream_wav_file(self):
        """Stream kokoro_test.wav via ROS topic"""
        self.get_logger().info('📡 Starting to stream kokoro_test.wav...')
        
        try:
            with wave.open(self.wav_file, 'rb') as wav:
                sample_rate = wav.getframerate()
                channels = wav.getnchannels()
                sample_width = wav.getsampwidth()
                n_frames = wav.getnframes()
                
                duration = n_frames / sample_rate
                
                self.get_logger().info(f'   Sample rate: {sample_rate}Hz')
                self.get_logger().info(f'   Channels: {channels}')
                self.get_logger().info(f'   Duration: {duration:.2f}s')
                self.get_logger().info(f'   Frames: {n_frames}')
                
                # Publish status for auto rate detection
                status_msg = String()
                status_msg.data = f'kokoro_{sample_rate}'
                self.status_pub.publish(status_msg)
                
                # Read and stream all audio data
                audio_data = wav.readframes(n_frames)
                
                self.get_logger().info(f'   Total audio bytes: {len(audio_data)}')
                
                # Stream in chunks
                chunks_sent = 0
                for i in range(0, len(audio_data), self.chunk_size):
                    chunk = audio_data[i:i + self.chunk_size]
                    
                    msg = UInt8MultiArray()
                    msg.data = list(chunk)
                    self.audio_pub.publish(msg)
                    
                    chunks_sent += 1
                    
                    # Small delay to simulate streaming
                    time.sleep(self.stream_delay)
                
                self.get_logger().info(f'✓ Streamed {chunks_sent} chunks ({len(audio_data)} bytes)')
                self.get_logger().info('   Audio should play on the receiver node...')
                
        except Exception as e:
            self.get_logger().error(f'Error streaming WAV: {e}')
        
        # Only stream once, then exit
        self.get_logger().info('Stream complete. Node will continue running.')


def main(args=None):
    rclpy.init(args=args)
    node = TestFFmpegStreamer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
