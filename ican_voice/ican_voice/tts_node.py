import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            'robot_speech',
            self.listener_callback,
            10)
        
        # Initialize the TTS engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150) # Speed of speech
        self.get_logger().info('TTS Node is ready. Publish to /robot_speech to talk.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Speaking: "{msg.data}"')
        self.engine.say(msg.data)
        self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()