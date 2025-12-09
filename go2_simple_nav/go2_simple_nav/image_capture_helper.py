# Example: Camera image capture for ROS2 node
# This is a helper for subscribing to a camera topic and storing the latest image
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageCaptureHelper:
    def __init__(self, node, image_topic='camera/image_raw'):
        self.bridge = CvBridge()
        self.latest_image = None
        node.create_subscription(Image, image_topic, self.image_callback, 10)

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_latest_image(self):
        return self.latest_image
