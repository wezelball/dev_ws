#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import json
import socket

class BatteryServiceNode(Node):
    def __init__(self):
        super().__init__("battery_service_node")
        # Declare and initialize parameters
        self.declare_parameter('raspberry_pi_ip', '192.168.1.172')
        self.declare_parameter('raspberry_pi_port', 6000)
        self.raspberry_pi_ip = self.get_parameter('raspberry_pi_ip').get_parameter_value().string_value
        self.raspberry_pi_port = self.get_parameter('raspberry_pi_port').get_parameter_value().integer_value

        # Create a ROS2 service to provide battery voltage
        self.srv = self.create_service(Trigger, 'get_battery_voltage', self.handle_battery_request)

        self.get_logger().info("battery service node has started")

    def handle_battery_request(self, request, response):
        """ Handle battery voltage request by querying the RPi """
        feedback = self.query_pi_for_feedback()

        if feedback:
            battery_voltage = feedback.get('battery_voltage', 'Unknown')
            response.success = True
            response.message = f'battery_voltage: {battery_voltage}V'
        else:
            response.success = False
            response.message = 'Failed to retrieve feedback from Raspberry Pi.'

        return response
    
    def query_pi_for_feedback(self):
        """Send a request to the Raspberry Pi to get feedback."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.raspberry_pi_ip, self.raspberry_pi_port))

                # Send a feedback request to the RPi
                request_data = {"command": "request_feedback"}
                s.sendall(json.dumps(request_data).encode('utf-8'))

                # receive feedback from the RPi
                data = s.recv(1024)
                feedback = json.loads(data.decode('utf-8'))
                self.get_logger().info(f'Received feedback: {feedback}')
                return feedback
        except Exception as e:
            self.get_logger().error(f'Error querying Raspberry Pi: {str(e)}')
            return None
    

def main(args=None):
    rclpy.init(args=args)
    battery_service_node = BatteryServiceNode()
    rclpy.spin(battery_service_node)
    battery_service_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()