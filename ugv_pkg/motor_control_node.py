#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import json
import socket

class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")
        self.declare_parameter('raspberry_pi_ip', '192.168.1.172')
        self.declare_parameter('raspberry_pi_port', 5000)
        self.raspberry_pi_ip = self.get_parameter('raspberry_pi_ip').get_parameter_value().string_value
        self.raspberry_pi_port = self.get_parameter('raspberry_pi_port').get_parameter_value().integer_value
        self.get_logger().info("motor control node subscriber has started")


    # Create a subscriber to listen to the 'cmd_vel' topic for motor commands
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        # Extract linear and angular velocity from the Twist message
        linear_velocity = msg.linear.x  # Forward/backward speed
        angular_velocity = msg.angular.z  # Left/right turning speed

        #self.get_logger().info("received Twist message from /cmd_vel topic")

        # Prepare the JSON command with time, linear, and angular velocities
        command = {
            "T": 13,  # 13 identifies this as ROS velocity command to robot
            "X": linear_velocity,
            "Z": angular_velocity
        }

        # Send command to Raspberry Pi
        self.send_command_to_pi(command)

    def send_command_to_pi(self, command):
        try:
            # Open a socket connection to the Raspberry Pi
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.raspberry_pi_ip, self.raspberry_pi_port))
                
                # Send the JSON command
                s.sendall(json.dumps(command).encode('utf-8'))
                self.get_logger().info(f'Sent command: {command}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command to Raspberry Pi: {str(e)}')



def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()