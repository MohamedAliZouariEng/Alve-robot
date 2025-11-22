import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge
from typing import List, Tuple
from std_msgs.msg import Bool

class BlobPointPublisher(Node):
    def __init__(self) -> None:
        super().__init__('blob_point_publisher')
        self.back_to_origin_sub = self.create_subscription(
            Bool,
            '/back_to_origin',
            self.back_to_origin_callback,
            10
        )
        self.subscription = self.create_subscription(
            Image,
            '/camera_two/image',  
            self.image_callback,
            10)
        self.final_sub = self.create_subscription(
            Bool,
            '/final',
            self.final_callback,
            10
        )
        self.publisher = self.create_publisher(Point, '/point_blob', 10)
        self.cv_bridge = CvBridge()
        self.process_image = False

    def final_callback(self, msg: Bool) -> None:
        """Callback for /final topic - shutdown node if True"""
        if msg.data:
            self.get_logger().info("Received final true - shutting down node")
            rclpy.shutdown()

    def back_to_origin_callback(self, msg: Bool) -> None:
        """Callback for /back_to_origin topic"""
        self.process_image = msg.data
        if self.process_image:
            self.get_logger().info("Received back_to_origin true - starting image processing")
        else:
            self.get_logger().info("Received back_to_origin false - stopping image processing")

    def image_callback(self, msg: Image) -> None:
        if not self.process_image:
            return  # Skip processing if flag is false
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error("Error converting ROS Image to OpenCV format: {0}".format(e))
            return

        # Convert BGR image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for green color in HSV
        lower_green = np.array([91, 251, 214])
        upper_green = np.array([255, 255, 255])

        # Create a binary mask for green color
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

       # Draw coordinate frame
        self.draw_frame(cv_image)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw a red circle around the blob contour
        for contour in contours:
            self.draw_blob_contour_circle(cv_image, contour)

        # If no contours are found, return
        if not contours:
            return

        # Find the largest contour
        blob_contour = max(contours, key=cv2.contourArea)

        # Calculate centroid of the contour
        moment = cv2.moments(blob_contour)
        if moment["m00"] != 0:             # Area of the object
            blob_x = float(moment["m10"] / moment["m00"])  # x-coordinate of centroid
            blob_y = float(moment["m01"] / moment["m00"])  # y-coordinate of centroid

            # x and y coordinates of the blob are computed relative to the center of the image
            x, y = self.get_blob_relative_position(cv_image, blob_x, blob_y)

            # Publish Blob
            self.publish_blob(x, y)

            # Draw detected blobs
            self.draw_keypoints(cv_image, [(blob_x, blob_y)])

        # Display the image (optional)
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

    def draw_frame(self,
                image: np.ndarray,
                dimension=0.1,      #- dimension relative to frame size
                line=2              #- line's thickness
        ) -> None:
        """ Draw X Y coordinate frame at the center of the image"""
        
        rows = image.shape[0]
        cols = image.shape[1]
        size = min([rows, cols])
        center_x = int(cols/2.0)
        center_y = int(rows/2.0)
        
        line_length = int(size*dimension)
        
        #-- X
        image = cv2.line(image, (center_x, center_y), (center_x+line_length, center_y), (0,0,255), line)
        #-- Y
        image = cv2.line(image, (center_x, center_y), (center_x, center_y+line_length), (0,255,0), line)
        

    def draw_blob_contour_circle(self, img: np.ndarray, contour: np.ndarray) -> None:
        """ Draw a red circle around the detected blob """
        # Get the bounding rectangle of the contour
        x, y, w, h = cv2.boundingRect(contour)
        
        # Calculate the center of the rectangle
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Calculate the radius of the circle based on the contour size
        radius = max(w, h) // 2

        # Draw a red circle around the center
        cv2.circle(img, (center_x, center_y), radius, (0, 0, 255), 2)


    def get_blob_relative_position(self, image: np.ndarray, x: float, y: float) -> Tuple[float, float]:
        """ Get blob position relative to the coordinate frame placed at the center of the image"""
        # The shape attribute of a NumPy array returns a tuple representing the dimensions of the array. 
        # For an image, the shape tuple consists of 3 elements: (height, width, channels).

        rows = float(image.shape[0]) # height
        cols = float(image.shape[1]) # width

        #  Coordinates of the center of the image 
        center_x    = 0.5*cols
        center_y    = 0.5*rows

        # The x and y coordinates of the keypoint are computed relative to the center of the image
        x = (x - center_x)/(center_x)
        y = (y - center_y)/(center_y)

        return x,y
    
    def publish_blob(self, x: float, y: float) -> None:
        """ Publish the blob position to /point_blob """
        # Create a Point message
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y

        # Publish the Point message
        self.publisher.publish(point_msg)


    def draw_keypoints(self, img: np.ndarray, keypoints: List[Tuple[float, float]]) -> None:
        """ Draw the detected blob keypoints in red """
        # Draw red circles at the detected blob keypoints
        for kp in keypoints:
            # Convert keypoints to integers
            kp = (int(kp[0]), int(kp[1]))
            cv2.circle(img, kp, 5, (0, 0, 255), -1)


def main(args=None) -> None:
    rclpy.init(args=args)
    blob_point_publisher = BlobPointPublisher()
    rclpy.spin(blob_point_publisher)
    blob_point_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()