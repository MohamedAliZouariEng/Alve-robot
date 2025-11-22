import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Dict, Any, List

class RangeDetector(Node):
    def __init__(self):
        super().__init__('range_detector')
        self.declare_parameter('filter', 'RGB')
        self.declare_parameter('preview', False)
        self.bridge = CvBridge()

        self.args = self.get_arguments()

        self.setup_trackbars()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image',  
            self.image_callback,
            10)

    def callback(self, value) -> None:
        pass

    def setup_trackbars(self) -> None:
        """ Creates OpenCV trackbars for adjusting the color range filter """

        cv2.namedWindow("Trackbars", 0)
        range_filter = self.args['filter'].upper()
        for i in ["MIN", "MAX"]:
            v = 0 if i == "MIN" else 255
            for j in range_filter:
                cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, self.callback)

    def get_arguments(self) -> Dict[str, Any]:
        """  Retrieves parameters set for color filtering and previewing """

        args = {}
        args['filter'] = self.get_parameter('filter').value
        args['preview'] = self.get_parameter('preview').value
        if not args['filter'].upper() in ['RGB', 'HSV']:
            self.get_logger().error("Please specify a correct filter.")
        return args

    def get_trackbar_values(self) -> List[int]:
        """ Retrieves the current values of the trackbars """

        values = []
        range_filter = self.args['filter'].upper()
        for i in ["MIN", "MAX"]:
            for j in range_filter:
                v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
                values.append(v)
        return values

    def image_callback(self, msg) -> None:

        # convert ROS Image message into a OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        range_filter = self.args['filter'].upper()

        if range_filter == 'RGB':
            frame_to_thresh = cv_image.copy()
        else:
            # convert cv_image to HSV color space
            frame_to_thresh = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # retrieves the trackbar values representing the minimum and maximum thresholds for each channel
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.get_trackbar_values()

        # Thresholding - Detect an object based on the range of pixel values in the HSV colorspace
        # It creates a binary image thresh where the pixels that fall within the specified range 
        # are white (255) and others are black (0)
        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        if self.args['preview']:
            # creates the preview image by performing a bitwise AND operation 
            # between the original image cv_image and itself using cv2.bitwise_and()
            # The mask param specifies that the operation should be applied only where thresh is non-zero (white)
            preview = cv2.bitwise_and(cv_image, cv_image, mask=thresh)
            # create preview image showing the result of the filtering process
            cv2.imshow("Preview", preview)
        else:
            cv2.imshow("Original", cv_image)
            cv2.imshow("Thresh", thresh)
        
        # Necessary for the OpenCV windows to remain open and responsive.
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    range_detector = RangeDetector()
    rclpy.spin(range_detector)
    range_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
