#!/usr/bin/env python3
"""
Test node: Save TTS audio stream to WAV file
Subscribes to /audio/tts_stream and saves to output.wav
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import wave
import os


class TTSToFileRecorder(Node):
    def __init__(self):
        super().__init__('tts_to_file_recorder')
        
        # Parameters
        self.declare_parameter('output_file', '/tmp/tts_output.wav')
        self.declare_parameter('sample_rate', 24000)  # Kokoro outputs 24kHz
        self.declare_parameter('channels', 1)
        self.declare_parameter('sample_width', 2)  # 16-bit
        
        self.output_file = self.get_parameter('output_file').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.sample_width = self.get_parameter('sample_width').value
        
        # Subscribe to TTS audio stream
        self.sub = self.create_subscription(
            UInt8MultiArray,
            'audio/tts_stream',
            self.audio_callback,
            10
        )
        
        # Audio buffer
        self.audio_chunks = []
        self.recording = False
        
        # Save timer (save after 2 seconds of silence)
        self.silence_counter = 0
        self.save_timer = self.create_timer(0.5, self.check_save)
        
        self.get_logger().info(f'Recording TTS audio to: {self.output_file}')
        self.get_logger().info('Listening on /audio/tts_stream...')
    
    def audio_callback(self, msg):
        """Accumulate audio chunks"""
        if msg.data:
            self.audio_chunks.extend(msg.data)
            self.recording = True
            self.silence_counter = 0
            
            # Log progress
            duration = len(self.audio_chunks) / (self.sample_rate * self.sample_width)
            self.get_logger().info(f'Recording... {duration:.1f}s', throttle_duration_sec=1.0)
    
    def check_save(self):
        """Check if we should save the file"""
        if self.recording:
            self.silence_counter += 1
            
            # Save after 4 timer ticks (2 seconds) of silence
            if self.silence_counter >= 4:
                self.save_audio()
                self.recording = False
                self.silence_counter = 0
    
    def save_audio(self):
        """Save accumulated audio to WAV file"""
        if not self.audio_chunks:
            return
        
        try:
            # Convert to bytes
            audio_bytes = bytes(self.audio_chunks)
            
            # Write WAV file
            with wave.open(self.output_file, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(self.sample_width)
                wf.setframerate(self.sample_rate)
                wf.writeframes(audio_bytes)
            
            duration = len(audio_bytes) / (self.sample_rate * self.sample_width)
            size_kb = len(audio_bytes) / 1024
            
            self.get_logger().info(f'✓ Saved {duration:.2f}s ({size_kb:.1f} KB) to: {self.output_file}')
            self.get_logger().info(f'Play with: aplay {self.output_file}')
            
            # Clear buffer for next recording
            self.audio_chunks.clear()
            
        except Exception as e:
            self.get_logger().error(f'Failed to save audio: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TTSToFileRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save on exit if we have data
        if node.audio_chunks:
            node.save_audio()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
