#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import json
import socket

class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")
        self.declare_parameter('raspberry_pi_ip', '192.168.1.172')
        self.declare_parameter('raspberry_pi_port', 5000)
        self.declare_parameter('enable_button_idx', 9)
        self.enable_button_index = self.get_parameter('enable_button_idx').get_parameter_value().integer_value
        self.raspberry_pi_ip = self.get_parameter('raspberry_pi_ip').get_parameter_value().string_value
        self.raspberry_pi_port = self.get_parameter('raspberry_pi_port').get_parameter_value().integer_value
        self.get_logger().info("motor control node subscriber has started")

        # Create a button state that will change only when enable button
        # transitions from ON to OFF
        self.enable_button_previous_state = 0
        self.enable_state = False

        # Create a subscriber to listen to the 'cmd_vel' topic for motor commands
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)
        # Create a subscriber to listen to the '/joy' topic for enble button detection
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        
    def listener_callback(self, msg):
        # Extract linear and angular velocity from the Twist message
        linear_velocity = msg.linear.x  # Forward/backward speed
        angular_velocity = msg.angular.z  # Left/right turning speed

        if self.enable_state:
            # Prepare the JSON command with time, linear, and angular velocities
            command = {
                "T": 13,  # 13 identifies this as ROS velocity command to robot
                "X": linear_velocity,
                "Z": angular_velocity
            }

            # Send command to Raspberry Pi
            self.send_command_to_pi(command)

    def joy_callback(self, msg):
        """Shuts the motor off if the enable button transition from 1 to 0"""
        if self.enable_button_previous_state == 1 and msg.buttons[self.enable_button_index] == 0:
            # Prepare the JSON command with time, linear, and angular velocities
            command = {
                "T": 13,  # 13 identifies this as ROS velocity command to robot
                "X": 0,
                "Z": 0
            }

            # Send command to Raspberry Pi
            self.send_command_to_pi(command)
        
        self.enable_button_previous_state = msg.buttons[self.enable_button_index]
        self.enable_state = True if msg.buttons[self.enable_button_index] == 1 else False

    def send_command_to_pi(self, command):
        try:
            # Open a socket connection to the Raspberry Pi
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.raspberry_pi_ip, self.raspberry_pi_port))
                
                # Send the JSON command
                s.sendall(json.dumps(command).encode('utf-8'))
                #self.get_logger().info(f'Sent motor command: {command}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command to Raspberry Pi: {str(e)}')



def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()