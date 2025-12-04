import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import pyaudio

class AudioStreamer(Node):
    def __init__(self):
        super().__init__('audio_streamer')
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'audio_stream', 10)
        
        # Audio Setup
        self.chunk_size = 4096 # Send bigger chunks for network efficiency
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=self.format,
                                  channels=self.channels,
                                  rate=self.rate,
                                  input=True,
                                  frames_per_buffer=self.chunk_size)
        
        # Timer to read audio continuously
        # 4096 frames @ 16000Hz is approx 0.25 seconds
        self.timer = self.create_timer(0.01, self.stream_callback)
        self.get_logger().info(f"Streaming Audio to /audio_stream at {self.rate}Hz...")

    def stream_callback(self):
        if self.stream.is_active():
            # Read raw bytes
            data = self.stream.read(self.chunk_size, exception_on_overflow=False)
            
            # Create ROS message
            msg = UInt8MultiArray()
            # Convert bytes to list of ints (required for UInt8MultiArray in Python)
            msg.data = list(data) 
            
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop_stream()
        node.stream.close()
        node.p.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()