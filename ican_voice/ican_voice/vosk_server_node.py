import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import os
import json
import urllib.request
import zipfile
from vosk import Model, KaldiRecognizer

class VoskServer(Node):
    def __init__(self):
        super().__init__('vosk_server')
        
        # Subscribe to audio, Publish text
        self.subscription = self.create_subscription(
            UInt8MultiArray, 
            'audio_stream', 
            self.audio_callback, 
            10)
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)

        # Initialize Vosk
        model_path = os.path.expanduser("~/vosk-model")
        if not os.path.exists(model_path):
            self.get_logger().warn(f"Model not found at {model_path}")
            self.get_logger().info("Downloading Vosk model...")
            if not self.download_vosk_model():
                self.get_logger().error("Failed to download model")
                return

        self.get_logger().info("Loading Vosk Model (Server Mode)...")
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.get_logger().info("Ready. Waiting for audio stream...")
    
    def download_vosk_model(self):
        """Download and extract Vosk model if not present"""
        try:
            home_dir = os.path.expanduser("~")
            model_url = "https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip"
            zip_path = os.path.join(home_dir, "vosk-model.zip")
            extracted_dir = os.path.join(home_dir, "vosk-model-small-en-us-0.15")
            final_dir = os.path.join(home_dir, "vosk-model")
            
            self.get_logger().info(f"Downloading from {model_url}...")
            urllib.request.urlretrieve(model_url, zip_path)
            
            self.get_logger().info("Extracting model...")
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                zip_ref.extractall(home_dir)
            
            # Rename to standard name
            if os.path.exists(extracted_dir):
                os.rename(extracted_dir, final_dir)
            
            # Clean up zip file
            os.remove(zip_path)
            
            self.get_logger().info(f"Model installed successfully at {final_dir}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to download model: {e}")
            return False

    def audio_callback(self, msg):
        # Convert list of ints back to raw bytes
        audio_data = bytes(msg.data)
        
        # Feed to Vosk
        if self.recognizer.AcceptWaveform(audio_data):
            result_json = self.recognizer.Result()
            result_dict = json.loads(result_json)
            text = result_dict.get('text', '')
            
            if text:
                self.get_logger().info(f"Recognized: {text}")
                pub_msg = String()
                pub_msg.data = text
                self.publisher_.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoskServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()