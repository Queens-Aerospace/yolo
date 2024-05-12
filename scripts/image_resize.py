import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/image', 10)
        self.bridge = CvBridge()

    def listener_callback(self, data):
        # Convert the incoming image from ROS message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgra8')
        
        # Convert BGRA to BGR (this drops the alpha channel)
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
        
        # Resize the image
        resized_image = cv2.resize(bgr_image, (608, 608))
        
        # Convert the processed image back to a ROS image message
        image_message = self.bridge.cv2_to_imgmsg(resized_image, encoding="bgr8")
        
        # Publish the image
        self.publisher.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
