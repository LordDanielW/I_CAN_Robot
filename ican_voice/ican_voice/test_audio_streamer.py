#!/usr/bin/env python3
"""
Test Audio Streamer Node - Subscribe and playback microphone stream
Listens to /audio_stream and plays it back through speakers for testing
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import pyaudio
import threading
from queue import Queue


class AudioStreamerTest(Node):
    def __init__(self):
        super().__init__('test_audio_streamer')
        
        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('buffer_size', 1024)
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.buffer_size = self.get_parameter('buffer_size').value
        
        # Audio queue for playback
        self.audio_queue = Queue(maxsize=50)
        
        # PyAudio setup for playback
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            output=True,
            frames_per_buffer=self.buffer_size
        )
        
        # Subscribe to audio stream
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'audio_stream',
            self.audio_callback,
            10
        )
        
        # Statistics
        self.packet_count = 0
        self.dropped_packets = 0
        
        # Playback thread
        self.running = True
        self.playback_thread = threading.Thread(target=self.playback_worker, daemon=True)
        self.playback_thread.start()
        
        # Statistics timer
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('🎤 Audio Streamer Test Node Started')
        self.get_logger().info(f'   Listening to: /audio_stream')
        self.get_logger().info(f'   Playback: {self.sample_rate}Hz, {self.channels} channel(s)')
        self.get_logger().info('   Speak into microphone - you should hear yourself!')
    
    def audio_callback(self, msg):
        """Receive audio data and queue for playback"""
        self.packet_count += 1
        
        try:
            audio_data = bytes(msg.data)
            self.audio_queue.put(audio_data, block=False)
        except:
            self.dropped_packets += 1
    
    def playback_worker(self):
        """Background thread for audio playback"""
        while self.running:
            try:
                audio_data = self.audio_queue.get(timeout=1.0)
                if audio_data and self.stream:
                    self.stream.write(audio_data)
            except:
                continue
    
    def print_stats(self):
        """Print statistics"""
        queue_size = self.audio_queue.qsize()
        self.get_logger().info(
            f'📊 Stats: Packets={self.packet_count}, '
            f'Dropped={self.dropped_packets}, '
            f'Queue={queue_size}/50'
        )
    
    def destroy_node(self):
        """Cleanup resources"""
        self.running = False
        
        self.get_logger().info('Shutting down audio playback...')
        
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        
        self.p.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamerTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
