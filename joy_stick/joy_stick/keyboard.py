#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import termios
import tty
import sys
import select

class KeyboardMecanumControl(Node):
    def __init__(self):
        super().__init__('keyboard_mecanum_control')
        
        self.publisher = self.create_publisher(
            TwistStamped, 
            '/mecanum_drive_controller/cmd_vel', 
            10
        )
        
        self.current_twist = TwistStamped()
        self.current_twist.header = Header()
        self.current_twist.header.frame_id = ''
        
        # Control parameters (matching your joystick node)
        self.linear_speed = 0.5
        self.angular_speed = 1.5
        
        self.get_logger().info("Keyboard Mecanum Control Initialized")
        self.get_logger().info("---------------------------")
        self.get_logger().info("Use these keys to control:")
        self.get_logger().info("  I: Move forward")
        self.get_logger().info("  ,: Move backward")
        self.get_logger().info("  J: Rotate left")
        self.get_logger().info("  L: Rotate right")
        self.get_logger().info("  K: Stop all movement")
        self.get_logger().info("Press CTRL+C to quit")
        
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'i':
                    self.current_twist.twist.linear.x = 0.0
                    self.current_twist.twist.linear.y = self.linear_speed
                    self.current_twist.twist.angular.z = 0.0
                elif key == ',':
                    self.current_twist.twist.linear.x = 0.0
                    self.current_twist.twist.linear.y = -self.linear_speed
                    self.current_twist.twist.angular.z = 0.0
                elif key == 'j':
                    self.current_twist.twist.linear.x = 0.0
                    self.current_twist.twist.linear.y = 0.0
                    self.current_twist.twist.angular.z = self.angular_speed
                elif key == 'l':
                    self.current_twist.twist.linear.x = 0.0
                    self.current_twist.twist.linear.y = 0.0
                    self.current_twist.twist.angular.z = -self.angular_speed
                elif key == 'k':
                    self.current_twist.twist.linear.x = 0.0
                    self.current_twist.twist.linear.y = 0.0
                    self.current_twist.twist.angular.z = 0.0
                
                if key in ['i', ',', 'j', 'l', 'k']:
                    self.current_twist.header.stamp = self.get_clock().now().to_msg()
                    self.publisher.publish(self.current_twist)
                    self.get_logger().info(f"Command - X: {self.current_twist.twist.linear.x:.2f}, "
                                         f"Y: {self.current_twist.twist.linear.y:.2f}, "
                                         f"Rot: {self.current_twist.twist.angular.z:.2f}")
                
                if key == '\x03':  # CTRL+C
                    break
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            # Stop the robot before exiting
            self.current_twist.twist.linear.x = 0.0
            self.current_twist.twist.linear.y = 0.0
            self.current_twist.twist.angular.z = 0.0
            self.current_twist.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.current_twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardMecanumControl()
    keyboard_control.run()
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()