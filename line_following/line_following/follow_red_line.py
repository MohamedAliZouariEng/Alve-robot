import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Bool
import os
import sys

class LineFollower(Node):

    def __init__(self) -> None:
        super().__init__('line_follower')
        self.bridge = CvBridge()

        self.camera_view_clear = False
        self.clear_subscription = self.create_subscription(Bool, '/camera_view_clear', self.clear_callback, 10)
        self.subscription = None
        
        # Subscribe to camera image
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.camera_callback, 10)
        
        self.pub_vel = self.create_publisher(TwistStamped, 'mecanum_drive_controller/cmd_vel', 10)
        self.pub_conveyor = self.create_publisher(Bool, '/forward_conveyor', 10)
        
        # Initialize color bounds
        self.lower_color = np.array([0, 0, 226])  # Default to white
        self.upper_color = np.array([255, 4, 255])  # Default to white
        self.current_object = "donut"  # Default object
        
        # Define color ranges
        self.color_ranges = {
            "orange": {
                "lower": np.array([0, 166, 0]),
                "upper": np.array([24, 170, 255])
            },
            "pizza": {
                "lower": np.array([22, 140, 0]),
                "upper": np.array([24, 170, 255])
            },
            "donut": {
                "lower": np.array([0, 0, 226]),
                "upper": np.array([255, 4, 255])
            }
        }
        
        # File path for reading object data
        #self.data_file_path = '/home/ubuntu24/ros2_ws/src/advanced_perception/data/data.txt'
        from pathlib import Path
        self.data_file_path = Path.home() / 'ros2_ws' / 'src' / 'advanced_perception' / 'data' / 'data.txt'
        
        # Timer to periodically check the data file
        self.timer = self.create_timer(1.0, self.check_data_file)  # Check every 1 second
        self.last_file_check_time = 0.0

    def clear_callback(self, msg: Bool) -> None:
        """Callback for camera view clear topic"""
        if msg.data and not self.camera_view_clear:
            self.camera_view_clear = True
            self.get_logger().info("Camera view clear - starting line following")
            # Now create the camera subscription
            self.subscription = self.create_subscription(
                Image, '/camera/image', self.camera_callback, 10)


    def check_data_file(self) -> None:
        """Periodically check the data.txt file for object updates"""
        try:
            # Check if file exists and has been modified recently
            if os.path.exists(self.data_file_path):
                # Read the object name from file
                with open(self.data_file_path, 'r') as f:
                    detected_object = f.read().strip().lower()
                
                # Only process if we got valid data
                if detected_object and detected_object in self.color_ranges:
                    self.update_object_tracking(detected_object)
                    
        except Exception as e:
            self.get_logger().warn(f"Error reading data file: {e}")

    def update_object_tracking(self, detected_object: str) -> None:
        """Update the object tracking based on detected object"""
        if detected_object != self.current_object:
            self.current_object = detected_object
            self.lower_color = self.color_ranges[detected_object]["lower"]
            self.upper_color = self.color_ranges[detected_object]["upper"]
            self.get_logger().info(f"Switched to {detected_object.upper()} color detection")

    def camera_callback(self, msg: Image) -> None:
        if not self.camera_view_clear:
            return
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().info(f"Error converting image: {e}")
            return

        height, width, _ = cv_image.shape
        rows_to_watch = 20
        # Crop image 
        crop_img = cv_image[height*3//4:height*3//4 + rows_to_watch][1:width]

        # Convert BGR image to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Create a binary mask using the current color bounds
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)

        # Calculate centroid of the mask
        moment = cv2.moments(mask, False)
        color_detected = True
        
        try:
            # x and y coordinates of centroid
            cx, cy = moment['m10'] / moment['m00'], moment['m01'] / moment['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
            color_detected = False 

        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        # Draw circle to visualize the detected centroid
        cv2.circle(res, (int(cx), int(cy)), 10, (255, 0, 0), -1)

        # Add text to show current object
        cv2.putText(res, f"Tracking: {self.current_object}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Display
        cv2.imshow("RES", res)
        cv2.waitKey(1)

        # Calculate horizontal error between the centroid and the center of the image
        error_x = cx - width / 2
        self.pub_velocities(error_x, color_detected)
        
    def pub_velocities(self, error: float, color_detected: bool) -> None:
        twist_object = TwistStamped()
        conveyor_msg = Bool()

        if not color_detected:
            twist_object.twist.linear.x = 0.0
            twist_object.twist.linear.y = 0.0
            twist_object.twist.angular.z = 0.0

            conveyor_msg.data = True
            self.get_logger().info(f"{self.current_object.capitalize()} line not detected - stopping robot and activating conveyor")
            self.pub_conveyor.publish(conveyor_msg)
            # Then shutdown the node
            self.get_logger().info("Shutting down line follower node")
            sys.exit(0)
        else:
            twist_object.twist.linear.x = 0.0
            twist_object.twist.linear.y = 0.5
            twist_object.twist.angular.z = -error / 100
            conveyor_msg.data = False
            self.pub_conveyor.publish(conveyor_msg)
            self.get_logger().info(f"Tracking {self.current_object} - Angular: {twist_object.twist.angular.z:.2f}, Linear: {twist_object.twist.linear.y:.2f}")
        
        self.pub_vel.publish(twist_object)
        



def main(args=None) -> None:
    rclpy.init(args=args)
    line_follower = LineFollower()
    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        pass
    finally:
        line_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()