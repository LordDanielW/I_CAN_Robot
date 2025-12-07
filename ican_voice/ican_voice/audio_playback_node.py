#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import pyaudio

class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'audio/tts_stream',
            self.audio_callback,
            10
        )
        
        # Audio Setup
        self.p = pyaudio.PyAudio()
        
        # CRITICAL: Piper 'medium' models usually output 22050Hz
        # If your voice sounds slow/fast, change this rate.
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=22050, 
            output=True
        )
        
        self.get_logger().info('Audio Player Ready (Linux). Waiting for stream...')

    def audio_callback(self, msg):
        try:
            # Convert list of ints back to bytes
            audio_data = bytes(msg.data)
            # Write directly to the audio device
            self.stream.write(audio_data)
        except Exception as e:
            self.get_logger().error(f'Playback error: {e}')

    def destroy_node(self):
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