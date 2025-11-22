#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthNormalizer(Node):
    def __init__(self):
        super().__init__('depth_normalizer')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/camera_two/depth_image',
            self.depth_callback,
            10)
        self.pub = self.create_publisher(Image, '/camera/depth_image_normalized', 10)
        
        # Parameters for depth scaling
        self.declare_parameter('min_depth', 0.1)  # Minimum depth in meters (adjust as needed)
        self.declare_parameter('max_depth', 10.0) # Maximum depth in meters (adjust as needed)
        self.declare_parameter('invert', False)   # Whether to invert the grayscale
        
    def depth_callback(self, msg):
        try:
            # Get parameters
            min_depth = self.get_parameter('min_depth').value
            max_depth = self.get_parameter('max_depth').value
            invert = self.get_parameter('invert').value
            
            # Convert to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Handle invalid values
            depth_image = np.nan_to_num(depth_image)
            valid_mask = (depth_image >= min_depth) & (depth_image <= max_depth)
            
            # Create normalized image
            normalized = np.zeros_like(depth_image, dtype=np.uint8)
            
            # Scale valid pixels to 0-255 range
            valid_pixels = depth_image[valid_mask]
            if len(valid_pixels) > 0:
                scaled = ((valid_pixels - min_depth) * (255.0 / (max_depth - min_depth))).astype(np.uint8)
                if invert:
                    scaled = 255 - scaled
                normalized[valid_mask] = scaled
            
            # Convert back to ROS message
            normalized_msg = self.bridge.cv2_to_imgmsg(normalized, encoding='mono8')
            self.pub.publish(normalized_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthNormalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()