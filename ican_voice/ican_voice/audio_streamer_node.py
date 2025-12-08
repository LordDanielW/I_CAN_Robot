#!/usr/bin/env python3
"""
Audio Streamer Node - Microphone capture and streaming
Captures audio from microphone and publishes to ROS topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import pyaudio
import os
import sys
from contextlib import contextmanager

@contextmanager
def suppress_alsa_errors():
    """Temporarily suppress ALSA error messages"""
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    try:
        os.dup2(devnull, 2)
        os.close(devnull)
        yield
    finally:
        os.dup2(old_stderr, 2)
        os.close(old_stderr)


class AudioStreamer(Node):
    def __init__(self):
        super().__init__('audio_streamer')
        
        # Parameters
        self.declare_parameter('sample_rate', 16000)  # Whisper expects 16kHz
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_size', 4096)  # ~256ms @ 16kHz
        self.declare_parameter('device_index', -1)  # -1 for default
        
        self.rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.chunk_size = self.get_parameter('chunk_size').value
        device_index = self.get_parameter('device_index').value
        
        # Publisher
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'audio_stream', 10)
        
        # Audio Setup (suppress ALSA warnings during initialization)
        with suppress_alsa_errors():
            self.p = pyaudio.PyAudio()
        
        # List available devices
        self.get_logger().info("Available audio input devices:")
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                self.get_logger().info(f"  [{i}] {info['name']}")
        
        # Open stream
        stream_kwargs = {
            'format': pyaudio.paInt16,
            'channels': self.channels,
            'rate': self.rate,
            'input': True,
            'frames_per_buffer': self.chunk_size,
        }
        
        if device_index >= 0:
            stream_kwargs['input_device_index'] = device_index
        
        try:
            self.stream = self.p.open(**stream_kwargs)
            self.get_logger().info(f"Opened audio stream @ {self.rate}Hz, {self.chunk_size} frames/chunk")
        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {e}")
            raise
        
        # Timer to read audio continuously
        # Read interval should be less than chunk duration to avoid buffer overflow
        chunk_duration = self.chunk_size / self.rate
        self.timer = self.create_timer(chunk_duration * 0.5, self.stream_callback)
        
        self.get_logger().info(f"Streaming audio to /audio_stream...")

    def stream_callback(self):
        """Read audio from microphone and publish"""
        if not self.stream.is_active():
            return
            
        try:
            # Read raw bytes from microphone
            data = self.stream.read(self.chunk_size, exception_on_overflow=False)
            
            # Create and publish ROS message
            msg = UInt8MultiArray()
            msg.data = list(data)
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f"Audio read error: {e}")

    def destroy_node(self):
        """Cleanup audio resources"""
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'p'):
            self.p.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()