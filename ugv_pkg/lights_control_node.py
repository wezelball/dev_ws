import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # Service type for turning lights on or off
import socket
import json

class LightsControlNode(Node):

    def __init__(self):
        super().__init__('lights_control_node')
        # Create a service that handles light control (turn on/off)
        self.srv = self.create_service(SetBool, 'toggle_lights', self.handle_toggle_lights)
        self.declare_parameter('raspberry_pi_ip', '192.168.1.172')
        self.declare_parameter('raspberry_pi_port', 5000)
        self.raspberry_pi_ip = self.get_parameter('raspberry_pi_ip').get_parameter_value().string_value
        self.raspberry_pi_port = self.get_parameter('raspberry_pi_port').get_parameter_value().integer_value
        self.get_logger().info('Lights control service is ready.')

        # Set up the connection to the Raspberry Pi (using a socket for JSON commands)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.raspberry_pi_ip, self.raspberry_pi_port))  # Connect to the correct IP and port

    def handle_toggle_lights(self, request, response):
        # Prepare the JSON command for lights control
        if request.data:  # If True, turn lights ON (255 brightness)
            command = {"T": 132, "IO4": 255, "IO5": 255}
            response.success = True
            response.message = "Lights turned ON"
        else:  # If False, turn lights OFF (0 brightness)
            command = {"T": 132, "IO4": 0, "IO5": 0}
            response.success = True
            response.message = "Lights turned OFF"

        # Send the JSON command to the Raspberry Pi's serial communication handler
        try:
            self.sock.sendall(json.dumps(command).encode())
            self.get_logger().info(f'Sent command: {command}')
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {str(e)}")
            response.success = False
            response.message = "Failed to control lights"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LightsControlNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()