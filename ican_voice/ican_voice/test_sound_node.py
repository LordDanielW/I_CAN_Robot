#!/usr/bin/env python3
"""
Simple Test Sound Node - Plays kokoro_test.wav on startup
"""

import rclpy
from rclpy.node import Node
import subprocess
import os


class TestSoundNode(Node):
    def __init__(self):
        super().__init__('test_sound_node')
        
        self.get_logger().info('Test Sound Node started')
        
        # Path to kokoro_test.wav
        self.wav_file = os.path.join(
            os.path.dirname(__file__), 
            'kokoro_test.wav'
        )
        
        if not os.path.exists(self.wav_file):
            self.get_logger().error(f'WAV file not found: {self.wav_file}')
            return
        
        self.get_logger().info(f'Using WAV file: {self.wav_file}')
        
        # Set Bluetooth to A2DP profile
        try:
            subprocess.run(['pactl', 'set-card-profile', 'bluez_card.41_42_F4_7A_02_A6', 'a2dp_sink'],
                         check=False, capture_output=True, timeout=2)
            self.get_logger().info('✓ Set Bluetooth to A2DP profile')
        except:
            self.get_logger().warn('Could not set A2DP profile')
        
        # Play sound on startup
        self.play_test_sound()
        
        # Create a timer to play sound every 10 seconds
        self.timer = self.create_timer(10.0, self.play_test_sound)
    
    def play_test_sound(self):
        """Play kokoro_test.wav file"""
        try:
            self.get_logger().info('Playing kokoro_test.wav...')
            
            # Play using PulseAudio
            result = subprocess.run(
                ['paplay', '--device=bluez_sink.41_42_F4_7A_02_A6.a2dp_sink', self.wav_file],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                self.get_logger().info('✓ WAV file played successfully!')
            else:
                # Try default device
                self.get_logger().warn(f'A2DP failed, trying default device...')
                result = subprocess.run(
                    ['paplay', self.wav_file],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode == 0:
                    self.get_logger().info('✓ WAV file played on default device')
                else:
                    self.get_logger().warn(f'Failed to play: {result.stderr}')
            
        except Exception as e:
            self.get_logger().error(f'Error playing sound: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TestSoundNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
