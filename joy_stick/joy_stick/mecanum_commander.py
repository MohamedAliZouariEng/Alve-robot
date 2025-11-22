#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class MecanumCommander(Node):
    def __init__(self):
        super().__init__('mecanum_commander')
        
        # Publisher for mecanum drive commands
        self.publisher = self.create_publisher(
            TwistStamped, 
            '/mecanum_drive_controller/cmd_vel', 
            10
        )
        
        # Joystick subscriber (single joystick with both controls)
        self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Control parameters
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.5
        self.deadzone = 0.1
        
        # Timer for continuous publishing
        self.time_interval = 0.02
        self.timer = self.create_timer(self.time_interval, self.timer_callback)
        
        # Current command values
        self.current_twist = TwistStamped()
        self.current_twist.header = Header()
        self.current_twist.header.frame_id = ''
        
    def joy_callback(self, msg):
        # Left stick (axes 0,1) for movement
        # Right stick (axes 2,3) - using axis 3 for rotation
        
        # Get joystick values with deadzone
        left_right = msg.axes[0] if abs(msg.axes[0]) > self.deadzone else 0.0
        forward_back = msg.axes[1] if abs(msg.axes[1]) > self.deadzone else 0.0
        rotate = msg.axes[3] if abs(msg.axes[3]) > self.deadzone else 0.0
        
        # Debug output
        self.get_logger().info(f"Joystick Input - X: {left_right:.2f}, Y: {forward_back:.2f}, Rot: {rotate:.2f}")
        
        # Scale to max speeds
        self.current_twist.twist.linear.x = left_right * self.max_linear_speed  # Left/Right
        self.current_twist.twist.linear.y = forward_back * self.max_linear_speed  # Forward/Back
        self.current_twist.twist.angular.z = -rotate * self.max_angular_speed  # Rotation
        
        # Z-axis always zero for mecanum
        self.current_twist.twist.linear.z = 0.0
        
    def timer_callback(self):
        # Update timestamp
        self.current_twist.header.stamp = self.get_clock().now().to_msg()
        
        # Debug output
        self.get_logger().info(f"Command - X: {self.current_twist.twist.linear.x:.2f}, "
                             f"Y: {self.current_twist.twist.linear.y:.2f}, "
                             f"Rot: {self.current_twist.twist.angular.z:.2f}")
        
        # Publish the current command
        self.publisher.publish(self.current_twist)

def main(args=None):
    rclpy.init(args=args)
    commander = MecanumCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()