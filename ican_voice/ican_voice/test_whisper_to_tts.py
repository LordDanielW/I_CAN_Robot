#!/usr/bin/env python3
"""
Test node: Echo speech back through TTS
Subscribes to /speech_text and republishes to /tts/speak
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WhisperToTTSEcho(Node):
    def __init__(self):
        super().__init__('whisper_to_tts_echo')
        
        # Subscribe to Whisper output
        self.sub = self.create_subscription(
            String,
            'speech_text',
            self.speech_callback,
            10
        )
        
        # Publish to TTS input
        self.pub = self.create_publisher(String, 'tts/speak', 10)
        
        self.get_logger().info('Echo node ready: /speech_text -> /tts/speak')
    
    def speech_callback(self, msg):
        """Echo recognized speech to TTS"""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f'Echoing: "{text}"')
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WhisperToTTSEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
