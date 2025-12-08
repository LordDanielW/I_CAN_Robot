#!/usr/bin/env python3
"""
Audio Playback Node - Multi-rate audio player
Supports both Kokoro (24kHz) and Piper (22050Hz) TTS outputs
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import pyaudio
import threading
from queue import Queue


class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        
        # Parameters
        self.declare_parameter('sample_rate', 24000)  # 24000 for Kokoro, 22050 for Piper
        self.declare_parameter('channels', 1)
        self.declare_parameter('format', 'int16')
        self.declare_parameter('buffer_size', 1024)
        self.declare_parameter('auto_rate_detect', True)  # Auto-detect from TTS status
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.auto_rate_detect = self.get_parameter('auto_rate_detect').value
        
        # Audio queue for smooth playback
        self.audio_queue = Queue(maxsize=100)
        
        # PyAudio setup
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.init_audio_stream()
        
        # Subscribers
        self.sub_audio = self.create_subscription(
            UInt8MultiArray,
            'audio/tts_stream',
            self.audio_callback,
            10
        )
        
        # Optional: Listen to TTS status for auto rate detection
        if self.auto_rate_detect:
            self.sub_status = self.create_subscription(
                String,
                'tts/status',
                self.status_callback,
                10
            )
        
        # Playback thread
        self.running = True
        self.playback_thread = threading.Thread(target=self.playback_worker, daemon=True)
        self.playback_thread.start()
        
        self.get_logger().info(f'Audio Player Ready @ {self.sample_rate}Hz. Listening on /audio/tts_stream...')
    
    def init_audio_stream(self):
        """Initialize or reinitialize audio stream"""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            output=True,
            frames_per_buffer=self.buffer_size
        )
        self.get_logger().info(f'Audio stream opened @ {self.sample_rate}Hz')
    
    def status_callback(self, msg):
        """Handle TTS status for auto sample rate detection"""
        status = msg.data
        
        new_rate = None
        if 'kokoro' in status.lower():
            new_rate = 24000
        elif 'piper' in status.lower():
            new_rate = 22050
        
        if new_rate and new_rate != self.sample_rate:
            self.sample_rate = new_rate
            self.get_logger().info(f'Auto-detected sample rate: {new_rate}Hz')
            self.init_audio_stream()
    
    def audio_callback(self, msg):
        """Queue incoming audio data"""
        try:
            audio_data = bytes(msg.data)
            self.audio_queue.put(audio_data, timeout=0.1)
        except:
            self.get_logger().warn('Audio queue full, dropping packet')
    
    def playback_worker(self):
        """Background thread for continuous playback"""
        while self.running:
            try:
                audio_data = self.audio_queue.get(timeout=1.0)
                if audio_data and self.stream:
                    self.stream.write(audio_data)
            except:
                continue
    
    def destroy_node(self):
        """Cleanup resources"""
        self.running = False
        
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        
        self.p.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()