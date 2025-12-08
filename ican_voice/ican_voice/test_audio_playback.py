#!/usr/bin/env python3
"""
Test Audio Playback Node - Generate and send test audio
Generates test tones and sends them to /audio/tts_stream for playback testing
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import numpy as np
import struct


class AudioPlaybackTest(Node):
    def __init__(self):
        super().__init__('test_audio_playback')
        
        # Parameters
        self.declare_parameter('sample_rate', 24000)
        self.declare_parameter('test_frequency', 440.0)  # A4 note
        self.declare_parameter('duration', 1.0)  # seconds
        self.declare_parameter('volume', 0.3)  # 0.0 to 1.0
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.frequency = self.get_parameter('test_frequency').value
        self.duration = self.get_parameter('duration').value
        self.volume = self.get_parameter('volume').value
        
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
        
        self.get_logger().info('🔊 Audio Playback Test Node Started')
        self.get_logger().info(f'   Publishing to: /audio/tts_stream')
        self.get_logger().info(f'   Sample rate: {self.sample_rate}Hz')
        self.get_logger().info(f'   Test tone: {self.frequency}Hz, {self.duration}s')
        self.get_logger().info('')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  1 - Play test tone')
        self.get_logger().info('  2 - Play ascending tones')
        self.get_logger().info('  3 - Play beep pattern')
        self.get_logger().info('  4 - Play white noise burst')
        self.get_logger().info('  q - Quit')
        
        # Timer for interactive menu
        self.menu_timer = self.create_timer(0.1, self.menu_loop)
        self.waiting_for_input = False
    
    def generate_tone(self, frequency, duration, volume=None):
        """Generate a sine wave tone"""
        if volume is None:
            volume = self.volume
        
        num_samples = int(self.sample_rate * duration)
        t = np.linspace(0, duration, num_samples, False)
        tone = np.sin(2 * np.pi * frequency * t) * volume
        
        # Convert to int16
        audio_int16 = (tone * 32767).astype(np.int16)
        return audio_int16.tobytes()
    
    def generate_white_noise(self, duration, volume=None):
        """Generate white noise"""
        if volume is None:
            volume = self.volume
        
        num_samples = int(self.sample_rate * duration)
        noise = np.random.uniform(-1, 1, num_samples) * volume
        
        # Convert to int16
        audio_int16 = (noise * 32767).astype(np.int16)
        return audio_int16.tobytes()
    
    def send_audio(self, audio_data, chunk_size=4096):
        """Send audio data in chunks"""
        # Send status message
        status_msg = String()
        status_msg.data = f'kokoro_ready_{self.sample_rate}'
        self.status_pub.publish(status_msg)
        
        # Send audio in chunks
        for i in range(0, len(audio_data), chunk_size):
            chunk = audio_data[i:i+chunk_size]
            msg = UInt8MultiArray()
            msg.data = list(chunk)
            self.audio_pub.publish(msg)
            
            # Small delay to simulate streaming
            rclpy.spin_once(self, timeout_sec=0.001)
    
    def play_test_tone(self):
        """Play single test tone"""
        self.get_logger().info(f'▶️  Playing {self.frequency}Hz tone for {self.duration}s...')
        audio = self.generate_tone(self.frequency, self.duration)
        self.send_audio(audio)
        self.get_logger().info('✓ Done!')
    
    def play_ascending_tones(self):
        """Play ascending scale"""
        self.get_logger().info('▶️  Playing ascending tones...')
        frequencies = [262, 294, 330, 349, 392, 440, 494, 523]  # C major scale
        
        for freq in frequencies:
            self.get_logger().info(f'   {freq}Hz')
            audio = self.generate_tone(freq, 0.3)
            self.send_audio(audio)
        
        self.get_logger().info('✓ Done!')
    
    def play_beep_pattern(self):
        """Play beep pattern"""
        self.get_logger().info('▶️  Playing beep pattern (3 beeps)...')
        
        for i in range(3):
            self.get_logger().info(f'   Beep {i+1}/3')
            audio = self.generate_tone(800, 0.2, volume=0.4)
            self.send_audio(audio)
            
            # Silence between beeps
            silence = np.zeros(int(self.sample_rate * 0.2), dtype=np.int16)
            self.send_audio(silence.tobytes())
        
        self.get_logger().info('✓ Done!')
    
    def play_white_noise(self):
        """Play white noise burst"""
        self.get_logger().info('▶️  Playing white noise burst...')
        audio = self.generate_white_noise(0.5, volume=0.2)
        self.send_audio(audio)
        self.get_logger().info('✓ Done!')
    
    def menu_loop(self):
        """Interactive menu"""
        if self.waiting_for_input:
            return
        
        self.waiting_for_input = True
        
        try:
            print('\nSelect test (1-4, q to quit): ', end='', flush=True)
            
            # Note: This is a simple approach. For production, consider using a separate input thread
            import select
            import sys
            
            # Check if input is available (non-blocking)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                choice = sys.stdin.readline().strip().lower()
                
                if choice == '1':
                    self.play_test_tone()
                elif choice == '2':
                    self.play_ascending_tones()
                elif choice == '3':
                    self.play_beep_pattern()
                elif choice == '4':
                    self.play_white_noise()
                elif choice == 'q':
                    self.get_logger().info('Exiting...')
                    rclpy.shutdown()
                elif choice:
                    self.get_logger().warn(f'Invalid choice: {choice}')
        
        except Exception as e:
            pass
        finally:
            self.waiting_for_input = False


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlaybackTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nKeyboard interrupt detected')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
