#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import numpy as np

class VideoStreamSubscriberNode(Node):
    def __init__(self):
        super().__init__("video_stream_subscriber_node")

        # Setup IP address and ports
        self.declare_parameter('raspberry_pi_ip', '192.168.1.172')
        self.declare_parameter('raspberry_pi_port', 8000)
        self.raspberry_pi_ip = self.get_parameter('raspberry_pi_ip').get_parameter_value().string_value
        self.raspberry_pi_port = self.get_parameter('raspberry_pi_port').get_parameter_value().integer_value
        self.get_logger().info("video stream subscriber node has started")

        #ROS2 publisher for the camera feed
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.br = CvBridge()

        # Connect to the Raspberry Pi's video stream server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.raspberry_pi_ip, self.raspberry_pi_port))

        # Start the video stream processing
        self.process_stream()

    def process_stream(self):
        while rclpy.ok():
            try:
                # Receive the frame size (4 bytes)
                frame_size = int.from_bytes(self.sock.recv(4), 'big')

                # Receive the actual frame data
                frame_data = b''
                while len(frame_data) < frame_size:
                    frame_data += self.sock.recv(frame_size - len(frame_data))

                # Decode the frame
                frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)

                if frame is not None:
                    # Convert to ROS Image message and publish
                    image_message = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.publisher_.publish(image_message)

                    # THE LINES BELOW ARE FOR TESTING ONLY, COMMENT OUT LATER
                    # *******************************************************
                    """
                    # Display the frame using OpenCV
                    cv2.imshow("Live Video Feed", frame)
                    
                    # Press 'q' to exit the video feed window
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    """
                    # *******************************************************
                    # THE LINES ABOVE ARE FOR TESTING ONLY, COMMENT OUT LATER

            except Exception as e:
                self.get_logger().error(f'Error processing stream: {e}')
    

def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()