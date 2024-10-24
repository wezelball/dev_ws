import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty
import socket
import json

GIMBAL_PORT = 5000
RASPBERRY_PI_IP = '192.168.1.172'

class GimbalControlNode(Node):
    def __init__(self):
        super().__init__('gimbal_control_node')

        # Subscriber for basic gimbal control
        self.basic_control_sub = self.create_subscription(
            Float32MultiArray,
            'gimbal_control_simple',
            self.basic_control_callback,
            10)

        # Subscriber for continuous gimbal control
        self.continuous_control_sub = self.create_subscription(
            Float32MultiArray,
            'gimbal_control_move',
            self.continuous_control_callback,
            10)

        # Subscriber for gimbal stop
        self.stop_control_sub = self.create_subscription(
            Empty,
            'gimbal_stop',
            self.stop_control_callback,
            10)

        # Connect to the Raspberry Pi
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((RASPBERRY_PI_IP, GIMBAL_PORT))

        self.get_logger().info("gimbal control subscriber node has started")

    def send_command(self, cmd):
        """Send a JSON command to the Raspberry Pi."""
        command_str = json.dumps(cmd)
        self.sock.sendall(command_str.encode('utf-8'))

    def basic_control_callback(self, msg):
        """Callback for basic gimbal control."""
        x, y = msg.data[0], msg.data[1]
        spd, acc = msg.data[2], msg.data[3]
        command = {"T": 133, "X": x, "Y": y, "SPD": spd, "ACC": acc}
        self.send_command(command)
        self.get_logger().info(f"Sent basic gimbal control: {command}")

    def continuous_control_callback(self, msg):
        """Callback for continuous gimbal control."""
        x, y, sx, sy = msg.data[0], msg.data[1], msg.data[2], msg.data[3]
        command = {"T": 134, "X": x, "Y": y, "SX": sx, "SY": sy}
        self.send_command(command)
        self.get_logger().info(f"Sent continuous gimbal control: {command}")

    def stop_control_callback(self, msg):
        """Callback to stop the gimbal."""
        command = {"T": 135}
        self.send_command(command)
        self.get_logger().info("Sent stop gimbal command")

    # TODO: Add CMD_GIMBAL_USER_CTRL

def main(args=None):
    rclpy.init(args=args)
    gimbal_control_node = GimbalControlNode()
    rclpy.spin(gimbal_control_node)

if __name__ == '__main__':
    main()