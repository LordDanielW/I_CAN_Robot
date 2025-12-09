import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class Qwen3VLNode(Node):
    def __init__(self):
        super().__init__('qwen3_vl_node')
        # Subscribe to trigger topic
        self.trigger_sub = self.create_subscription(
            Bool,
            'trigger_capture',
            self.trigger_callback,
            10
        )


        # Image capture helper
        from .image_capture_helper import ImageCaptureHelper
        self.image_helper = ImageCaptureHelper(self)

        # Ollama helper
        from .ollama_helper import run_ollama_qwen3_vl
        self.run_ollama = run_ollama_qwen3_vl

        # Publisher for output string topic
        from std_msgs.msg import String
        self.output_pub = self.create_publisher(String, 'vlm_output', 10)

    def trigger_callback(self, msg):
        if msg.data:
            self.get_logger().info('Trigger received!')
            image = self.image_helper.get_latest_image()
            if image is not None:
                self.get_logger().info('Image captured.')
                # Run image through Ollama Qwen3-VL:8B
                result = self.run_ollama(image)
                self.get_logger().info(f'Ollama output: {result}')
                msg_out = String()
                msg_out.data = result
                self.output_pub.publish(msg_out)
            else:
                self.get_logger().warn('No image available yet.')


def main(args=None):
    rclpy.init(args=args)
    node = Qwen3VLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
