#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray
import subprocess
import os

class TTSStreamer(Node):
    def __init__(self):
        super().__init__('tts_streamer')
        
        # PARAMETERS
        # Ensure this points to the correct ONNX file on your WSL system
        self.declare_parameter('model_path', '/home/fire/piper_voices/en_US-lessac-medium.onnx') 
        self.model_path = self.get_parameter('model_path').value
        
        # Subscribe to text commands
        self.sub_text = self.create_subscription(String, 'tts/speak', self.speak_callback, 10)
        
        # Publish raw audio bytes
        self.pub_audio = self.create_publisher(UInt8MultiArray, 'audio/tts_stream', 10)
        
        self.get_logger().info('TTS Streamer Ready (WSL). Listening on /tts/speak...')

    def speak_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'Generating audio for: "{text}"')
        
        # Piper Command: Output raw 16-bit PCM to stdout
        cmd = [
            'piper',
            '--model', self.model_path,
            '--output-raw'
        ]
        
        try:
            # Open subprocess and write text to its stdin
            process = subprocess.Popen(
                cmd, 
                stdin=subprocess.PIPE, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE
            )
            
            # Send text and get raw audio data
            # For very long text, you might want to stream chunks, 
            # but for sentences, communicate() is fine.
            audio_data, stderr_data = process.communicate(input=text.encode('utf-8'))
            
            if process.returncode != 0:
                self.get_logger().error(f"Piper failed: {stderr_data.decode()}")
                return

            # Publish audio in chunks (optional, but good for networking)
            # Here we send it all at once for simplicity, or you can chunk it.
            # Let's chunk it to be safe for UDP/ROS networks.
            chunk_size = 4096
            for i in range(0, len(audio_data), chunk_size):
                chunk = audio_data[i:i+chunk_size]
                msg = UInt8MultiArray()
                msg.data = list(chunk)
                self.pub_audio.publish(msg)
                
            self.get_logger().info(f'Sent {len(audio_data)} bytes of audio.')

        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TTSStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()