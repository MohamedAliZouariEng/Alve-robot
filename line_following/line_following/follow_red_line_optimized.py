import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped

class LineFollowerOptimized(Node):

    def __init__(self) -> None:
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image', self.camera_callback, 10)
        self.pub_vel = self.create_publisher(TwistStamped, 'mecanum_drive_controller/cmd_vel', 10)

    def camera_callback(self, msg: Image) -> None:
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().info(f"Error converting image: {e}")
            return

        height, width, _ = cv_image.shape
        rows_to_watch = 20
        # Crop image - Fixed indexing
        crop_img = cv_image[height*3//4:height*3//4 + rows_to_watch, 1:width]

        # Convert BGR image to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red color in HSV
        lower_red = np.array([0, 4, 255])
        upper_red = np.array([0, 255, 255])

        # Create a binary mask for red color
        mask = cv2.inRange(hsv, lower_red, upper_red)

        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        self.get_logger().info("Number of centroids==>"+str(len(contours)))
        
        # Initialize variables
        cx = width / 2
        cy = height / 2
        centres = []
        
        # Calculate centroids for each contour
        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            try:
                # append centroid coordinates for each contour
                centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
                # draw green circle to visualize detected centroids
                cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)
            except ZeroDivisionError:
                pass

        # Only process if we found centroids
        if centres:
            # Retrieves appropriate centroid
            most_right_centroid = max(centres, key=lambda x: x[0])
            
            # retrieve selected centroid coordinates
            cx = most_right_centroid[0]
            cy = most_right_centroid[1]
            
            # draw red circle to visualize selected centroid
            cv2.circle(res, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        # Display
        cv2.imshow("RES", res)
        cv2.waitKey(1)

        # Calculate horizontal error between the centroid and the center of the image
        error_x = cx - width / 2

        # Publish velocities
        self.pub_velocities(error_x)


    def pub_velocities(self, error: float) -> None:
        twist_object = TwistStamped()
        twist_object.twist.linear.x = 0.0
        twist_object.twist.linear.y = 0.5
        twist_object.twist.angular.z = -error / 100 
        """
        if twist_object.twist.angular.z < -0.2:
            twist_object.twist.angular.z = -0.2
        elif twist_object.twist.angular.z >= 0.2:
            twist_object.twist.angular.z = 0.2
        """
        self.get_logger().info(f"Adjusting the angular velocity ---> {twist_object.twist.angular.z:.2f}\nAdjusting the linear velocity ---> {twist_object.twist.linear.y:.2f}" )
        self.pub_vel.publish(twist_object)


def main(args=None) -> None:
    rclpy.init(args=args)
    line_follower = LineFollowerOptimized()
    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        pass
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()