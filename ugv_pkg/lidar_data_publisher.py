import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import json

class LidarDataPublisher(Node):
    def __init__(self):
        super().__init__('lidar_data_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/lidar_data', 10)
        
        # Socket connection to the Raspberry Pi LiDAR server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('192.168.1.172', 9000))  # Replace with actual IP address
        self.get_logger().info("Connected to Raspberry Pi LiDAR server.")

        self.timer = self.create_timer(0.1, self.receive_lidar_data)

    def receive_lidar_data(self):
        try:
            data = self.sock.recv(1024)  # Adjust buffer size as needed
            if data:
                #print(f"Data received: {data}")
                lidar_points = json.loads(data.decode())
                # Create and populate the LaserScan message
                scan_msg = LaserScan()
                # Populate fields (like angle_min, angle_max, etc.) based on the LiDAR specs and data

                scan_msg.header.stamp = self.get_clock().now().to_msg()
                scan_msg.header.frame_id = "lidar_frame"  # Replace as needed
                scan_msg.ranges = [point[1] / 1000.0 for point in lidar_points]
                #print(f"Publishing LiDAR data: {scan_msg}")  # Example distance in meters
                self.publisher_.publish(scan_msg)
        except Exception as e:
            self.get_logger().error(f"Error receiving LiDAR data: {e}")

def main(args=None):
    rclpy.init(args=args)
    lidar_data_publisher = LidarDataPublisher()
    rclpy.spin(lidar_data_publisher)
    lidar_data_publisher.sock.close()
    lidar_data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()