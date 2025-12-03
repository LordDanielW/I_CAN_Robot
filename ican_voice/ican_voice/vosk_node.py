import os
import json
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

class VoskNode(Node):
    def __init__(self):
        super().__init__('vosk_node')
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        
        # Path to your model folder
        model_path = os.path.expanduser("~/vosk-model")
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at {model_path}. Please download it.")
            return

        self.get_logger().info("Loading Vosk Model...")
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        
        # Audio setup
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
        self.stream.start_stream()
        
        self.timer = self.create_timer(0.01, self.listen_callback)
        self.get_logger().info("Vosk listening... (Offline)")

    def listen_callback(self):
        # Read data from microphone
        data = self.stream.read(4000, exception_on_overflow=False)
        
        if self.recognizer.AcceptWaveform(data):
            result_json = self.recognizer.Result()
            result_dict = json.loads(result_json)
            text = result_dict.get('text', '')
            
            if text:
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
                self.get_logger().info(f"Detected: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = VoskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()