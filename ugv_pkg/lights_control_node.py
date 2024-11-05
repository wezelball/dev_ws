import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import json
import time

class LightsControlNode(Node):

    def __init__(self):
        super().__init__('lights_control_node')
        
        # Initialize the LED state and button configuration
        self.led_on = False
        self.toggle_button_index = 0  # X button on PS4 controller

        # Set up socket for sending commands (adjust IP and PORT as needed)
        self.host = '192.168.1.172'
        self.port = 5500  # Replace with the correct port for your setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))

        # Subscribe to joystick messages
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Set up debounce time parameter, time in seconds
        self.declare_parameter('debounce_time', 0.5)
        self.debounce_interval = self.get_parameter('debounce_time').get_parameter_value().double_value
        
        # Initialize LED state and debounce timestamp
        self.led_state = False
        self.last_toggle_time = 0.0

        self.get_logger().info("lights control node subscriber has started")

    def joy_callback(self, msg):
        
        # Debounce chack
        current_time = time.time()
        if current_time - self.last_toggle_time < self.debounce_interval:
            return
        
        # Update last toggle time
        self.last_toggle_time = current_time

        # Check if the toggle button is pressed
        if msg.buttons[self.toggle_button_index] == 1:
            self.toggle_leds()

    def toggle_leds(self):
        # Toggle LED state
        self.led_on = not self.led_on
        command = {"T": 132, "IO4": 255 if self.led_on else 0, "IO5": 255 if self.led_on else 0}
        #self.get_logger().info(f'LED command to be sent: {command}')
        # Send the command over the socket
        try:
            self.sock.sendall(json.dumps(command).encode('utf-8'))
            self.get_logger().info(f'LEDs {"ON" if self.led_on else "OFF"}')
        except Exception as e:
            self.get_logger().error(f'Failed to send LED command: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LightsControlNode()
    rclpy.spin(node)
    node.sock.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()