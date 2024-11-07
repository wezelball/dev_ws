import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import json

class LidarDataPublisher(Node):
    def __init__(self):
        super().__init__('lidar_data_publisher')
        self.lidar_pub = self.create_publisher(LaserScan, '/lidar_data', 10)
        
        # Socket connection to the Raspberry Pi LiDAR server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('192.168.1.172', 9000))  # Replace with actual IP address
        self.get_logger().info("Connected to Raspberry Pi LiDAR server.")

        self.timer = self.create_timer(0.1, self.read_lidar_data)

    def read_lidar_data(self):
        try:
            # Receive JSON data from the server
            data = self.sock.recv(4096).decode()  # Adjust buffer size if needed
            lidar_packet = json.loads(data)
            
            # Process and publish the LaserScan message
            self.publish_lidar_data(lidar_packet)
        
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON from server.")
        except Exception as e:
            self.get_logger().error(f"Error in read_lidar_data: {e}")

    def publish_lidar_data(self, lidar_packet):
        # Create and populate LaserScan message
        scan_msg = LaserScan()
        
        # Set header with timestamp
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'lidar_frame'

        # Extract angle_min and angle_max from the packet
        scan_msg.angle_min = float(lidar_packet.get("angle_min", 0.0))
        scan_msg.angle_max = float(lidar_packet.get("angle_max", 0.0))

        # Calculate angle_increment
        num_ranges = len(lidar_packet["ranges"])
        if num_ranges > 1:
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (num_ranges - 1)
        else:
            scan_msg.angle_increment = 0.0

        # Populate ranges with distance values
        scan_msg.ranges = [float(distance) for _, distance in lidar_packet["ranges"]]

        # Populate range_min and range_max as per the scanner's specifications
        scan_msg.range_min = 0.02  # Minimum valid distance (e.g., 2 cm)
        scan_msg.range_max = 4.0   # Maximum valid distance (e.g., 4 meters)

        # Publish the LaserScan message
        self.lidar_pub.publish(scan_msg)
        self.get_logger().info(f"Published LaserScan with {len(scan_msg.ranges)} points")
 
def main(args=None):
    rclpy.init(args=args)
    lidar_data_publisher = LidarDataPublisher()
    rclpy.spin(lidar_data_publisher)
    lidar_data_publisher.sock.close()
    lidar_data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()