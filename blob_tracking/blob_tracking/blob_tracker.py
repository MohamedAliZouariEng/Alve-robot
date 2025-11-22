import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TwistStamped
from std_msgs.msg import Bool
class BlobTracker(Node):
     
    def __init__(self) -> None:
        super().__init__('blob_tracker')
        use_sim_time = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.parameter.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([use_sim_time])
        # Add this subscription
        self.back_to_origin_sub = self.create_subscription(
            Bool,
            '/back_to_origin',
            self.back_to_origin_callback,
            10
        )
        self.subscription = self.create_subscription(
            Point,
            '/point_blob',
            self.blob_callback,
            10)
        self.final_pub = self.create_publisher(Bool, '/final', 10)
        self.publisher = self.create_publisher(TwistStamped, '/mecanum_drive_controller/cmd_vel', 10)

        self.angular_gain = 1.0
        self.no_blob_detected = True
        #self.timer = self.create_timer(1.0, self.rotate_continuous)
        # Add this flag to control processing
        self.process_blob = False

        self.executing_trajectory = False
        self.trajectory_start_time = None
        self.trajectory_stage = 0  # 0: not started, 1: moving forward, 2: turning left

    def back_to_origin_callback(self, msg: Bool) -> None:
        """Callback for /back_to_origin topic"""
        self.process_blob = msg.data
        if self.process_blob:
            self.get_logger().info("Received back_to_origin true - starting blob tracking")
        else:
            self.get_logger().info("Received back_to_origin false - stopping blob tracking")
            


    def blob_callback(self, msg: Point) -> None:
        if not self.process_blob:
            return  
        if self.executing_trajectory:
            return
        # Extract blob coordinates
        blob_x = msg.x
        blob_y = msg.y


        # Define linear velocity
        linear_x = 0.5

        if blob_x != 0.0 or blob_y != 0.0:
            # Adjust angular velocity based on blob's position
            angular_vel = -self.angular_gain * blob_x
            # Clip angular velocity to [-1, 1] range
            angular_vel = max(min(angular_vel, 1.5), -1.5)

            # Adjust linear velocity based on blob's position
            if blob_y >= 0.9:
                final_msg = Bool()
                final_msg.data = True
                self.final_pub.publish(final_msg)
                self.get_logger().info("Published True to /final topic")
                self.complete_trajectory()
                return  # Exit the callback completely

            # If we get here, blob is still approaching
            linear_vel = linear_x
            self.no_blob_detected = False
        else:
            return

        self.pub_velocities(linear_vel, angular_vel)
    '''
    def rotate_continuous(self) -> None:
        if not self.process_blob or self.executing_trajectory:
            return 
        # Rotate continuously
        if self.no_blob_detected:
            twist_msg = TwistStamped()
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.angular.z = 1.5  # Adjust angular velocity as needed
            self.publisher.publish(twist_msg)
            self.get_logger().warn("No blobs detected yet ... Rotating continuously")
    '''


    def execute_trajectory_step(self) -> None:
        """Execute one step of the trajectory - called by timer"""
        if not self.executing_trajectory:
            self.trajectory_timer.cancel()
            return
            
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.trajectory_start_time).nanoseconds / 1e9
        
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.trajectory_stage == 1:  # Moving forward stage
            if elapsed_time < 5.6:  # 3 seconds forward
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.linear.y = 0.5
                twist_msg.twist.angular.z = 0.0
                self.get_logger().info(f"Moving forward: {elapsed_time:.1f}/5.6s")
            else:
                self.trajectory_stage = 2
                self.trajectory_start_time = self.get_clock().now()
                self.get_logger().info("Finished moving forward, starting left turn")
                return
        
        elif self.trajectory_stage == 2:  # Turning left stage
            if elapsed_time < 1.2:  # 2 seconds turning
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.linear.y = 0.0
                twist_msg.twist.angular.z = 1.5
                self.get_logger().info(f"Turning left: {elapsed_time:.1f}/1.2s")
            else:
                self.trajectory_stage = 3
                self.trajectory_start_time = self.get_clock().now()
                self.get_logger().info("Finished moving forward, starting left turn")
                return
        elif self.trajectory_stage == 3:  
            if elapsed_time < 0.5:  # 2 seconds turning
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.linear.y = 0.5
                twist_msg.twist.angular.z = 0.0
                self.get_logger().info(f"Move forward: {elapsed_time:.1f}/0.5s")
            else:
                # Trajectory complete
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.linear.y = 0.0
                twist_msg.twist.angular.z = 0.0
                self.executing_trajectory = False
                self.trajectory_stage = 0
                self.get_logger().info("Trajectory complete! Robot stopped.")
                self.process_blob = False
                self.trajectory_timer.cancel()
        
        self.publisher.publish(twist_msg)

    def complete_trajectory(self) -> None:
        """Execute trajectory: move forward 3s, then turn left 2s"""
        if not self.executing_trajectory:
            self.get_logger().info("Starting complete trajectory: forward 3s, then left turn 2s")
            self.executing_trajectory = True
            self.trajectory_start_time = self.get_clock().now()
            self.trajectory_stage = 1  # Start with moving forward
            # Create a timer to continuously execute the trajectory
            self.trajectory_timer = self.create_timer(0.1, self.execute_trajectory_step)

    def pub_velocities(self, linear: float, angular: float) -> None:
        # Create Twist message with linear and angular velocities
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = linear
        twist_msg.twist.angular.z = angular

        # Publish the Twist message
        self.publisher.publish(twist_msg)
        self.get_logger().info(f"Linear vel: {linear:.2f}, Angular vel: {angular:.2f}")


def main(args=None) -> None:
    rclpy.init(args=args)
    blob_tracker = BlobTracker()
    rclpy.spin(blob_tracker)
    blob_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



