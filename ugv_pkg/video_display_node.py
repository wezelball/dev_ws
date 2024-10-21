#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoDisplayNode(Node):
    def __init__(self):
        super().__init__("video_display_node")
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',  # Subscribing to the video feed topic
            self.listener_callback,
            10)
        
        # Initialize the CvBridge here in the constructor
        self.br = CvBridge()
        
    def listener_callback(self, data):
        """Callback function to receive and display the video frames."""
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(data, 'bgr8')

        # Display the frame using OpenCV
        cv2.imshow("Live Video Feed", frame)

        # Press 'q' to exit the video feed window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
    

def main(args=None):
    rclpy.init(args=args)
    node = VideoDisplayNode()			# Modify name
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()