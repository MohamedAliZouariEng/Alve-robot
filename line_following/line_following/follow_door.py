import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
import time
from typing import List, Tuple

class DoorFollower(Node):
    def __init__(self) -> None:
        super().__init__('door_follower')
        self.bridge = CvBridge()
        self.door_choice_sub = self.create_subscription(
            String,
            '/door_choice',
            self.door_callback,
            10
        )
        self.img_sub = self.create_subscription(Image, '/camera/image', self.camera_callback, 10)
        self.pub_vel = self.create_publisher(TwistStamped, 'mecanum_drive_controller/cmd_vel', 10)
        self.door_choice = None
        self.door_detected = False
        self.last_detection_time = None


    def door_callback(self, msg: String) -> None:
        self.door_choice = msg.data

    def camera_callback(self, msg: Image) -> None:
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().info(f"Error converting image: {e}")
            return

        # Define lower and upper bounds for red, yellow and green colors in HSV
        red_bounds = ([0, 4, 255], [0, 255, 255])
        green_bounds = ([49, 255, 255], [91, 255, 255])
        yellow_bounds = ([34, 255, 0], [82, 255, 255])
        
        # Define height and width of image
        height, width, _ = cv_image.shape
        # Crop images
        crop_img, crop_top_img = self.crop_image(cv_image, height, width)
        # Convert to HSV
        hsv, hsv_detect_door = self.convert_to_hsv(crop_img, crop_top_img)         

        # Detects red contours
        res, red_contours = self.detect_contours(hsv, red_bounds[0], red_bounds[1], crop_img)

        if self.door_choice == 'green':
            # Detects green contours
            res2, green_contours = self.detect_contours(hsv_detect_door, green_bounds[0], green_bounds[1], crop_top_img)
            # Checks if green door has been detected
            self.door_detected = self.search_door('green', green_contours, res2)

        if self.door_choice == 'yellow':
            # Detects yellow contours
            res2, yellow_contours = self.detect_contours(hsv_detect_door, yellow_bounds[0], yellow_bounds[1], crop_top_img)
            # Checks if yellow door has been detected
            self.door_detected = self.search_door('yellow', yellow_contours, res2)

        # Calculate centroids for each red contours
        centres = []
        for i in range(len(red_contours)):
            moments = cv2.moments(red_contours[i])
            try:
                # append centroid coordinates for each contour
                centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
                # draw yellow circle to visualize detected centroids
                cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)
            except ZeroDivisionError:
                pass

        # checks if arrived at the door destination
        if len(centres) < 1 and self.door_detected:
            self.stop_robot()
            return  

        # Retrieves appropriate centroid
        most_right_centroid = max(centres, key=lambda x: x[0])
        most_left_centroid = min(centres, key=lambda x: x[0])

        # Checks if the door choice color choice is valid
        self.is_valid_door_choice()
            
        # Calculate centroids coordinates
        if (self.is_valid_door_choice() and self.door_detected):
            cx, cy = self.get_centroid_coordinates(most_left_centroid, height, width)

        else:
            cx, cy = self.get_centroid_coordinates(most_right_centroid, height, width)
    
        # draw red circle to visualize selected centroid
        cv2.circle(res, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        # Display
        cv2.imshow("RES", res)
        cv2.waitKey(1)

        # Calculate horizontal error between the centroid and the center of the image
        error_x = cx - width / 2

        # Publish velocities
        self.pub_velocities(error_x)

    def crop_image(self, cv_image: np.ndarray, height: int, width: int) -> Tuple[np.ndarray, np.ndarray]:
        """ Crop image """
        rows_to_watch = 20
        cols_to_watch = 20
        crop_img = cv_image[height*3//4:height*3//4 + rows_to_watch][1:width]
        crop_top_img = cv_image[1:cols_to_watch][1:width]
        return crop_img, crop_top_img

    def convert_to_hsv(self, crop_img: np.ndarray, crop_top_img: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """ Converts from BGR to HSV """
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        hsv_detect_door = cv2.cvtColor(crop_top_img, cv2.COLOR_BGR2HSV)
        return hsv, hsv_detect_door

    def detect_contours(self, hsv: np.ndarray, lower_bound: List[int], upper_bound: List[int], image: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """ Detects for contours based on HSV range values """
        mask = cv2.inRange(hsv, np.array(lower_bound), np.array(upper_bound))
        res = cv2.bitwise_and(image, image, mask=mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        return res, contours

    def is_recently_detected(self) -> bool:
        """ Determines if door was recently detected """
        if self.last_detection_time is not None:
            current_time = time.time()
            elapsed_time = current_time - self.last_detection_time
            if elapsed_time < 60:
                return True
        return False

    def search_door(self, door_color: str, contours: List[np.ndarray], res: np.ndarray) -> bool:
        """ Searches for a specific colored door """
        if contours:
            self.get_logger().warning(f'{door_color.capitalize()} door detected')
            self.door_detected = True
            # Set timer when the contour has been detected
            self.last_detection_time = time.time()
            cv2.imshow("RES2", res)
            cv2.waitKey(1)
        else:
            if self.is_recently_detected():
                self.door_detected = True
            else:
                self.get_logger().warning(f'Still searching for the {door_color} door')
                self.door_detected = False

        return self.door_detected

    def is_valid_door_choice(self) -> bool:
        """ Checks if the door color choice is valid """
        valid_door_colors = ['green', 'yellow']
        if self.door_choice in valid_door_colors:
            return True
        else:
            self.get_logger().warning("Please choose between 'green' and 'yellow' for the door color.")
            return False

    def get_centroid_coordinates(self, centroid: Tuple[int, int], height: int, width: int) -> Tuple[int, int]:
        """ Retrieves centroid's x and y coorinates"""
        try:
            # retrieve selected centroid coordinates
            cx = centroid[0]
            cy = centroid[1]
        except:
            cy, cx = height/2, width/2

        return cx, cy
        
    def stop_robot(self) -> None:
        """ Stops the robot movement """
        twist_object = TwistStamped()
        twist_object.twist.linear.x = 0.0
        twist_object.twist.linear.y = 0.0
        twist_object.twist.angular.z  = 0.0
        self.pub_vel.publish(twist_object)

    def pub_velocities(self, error: float) -> None:
        """ Publish command velocities """
        twist_object = TwistStamped()
        twist_object.twist.linear.y  = 0.5
        angular_velocity = -error / 100

        # Ensure angular velocity is within [-0.3, 0.3]
        if angular_velocity < -0.3:
            angular_velocity = -0.3
        elif angular_velocity >= 0.3:
            angular_velocity = 0.3

        twist_object.twist.angular.z = angular_velocity
        self.get_logger().info(f"Adjusting the angular velocity ---> {twist_object.angular.z:.2f}" )
        self.pub_vel.publish(twist_object)

def main(args=None) -> None:
    rclpy.init(args=args)
    door_follower = DoorFollower()
    try:
        rclpy.spin(door_follower)
    except KeyboardInterrupt:
        pass
    door_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()