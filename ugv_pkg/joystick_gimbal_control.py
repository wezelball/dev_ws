import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class JoystickGimbalControl(Node):
    def __init__(self):
        super().__init__('joystick_gimbal_control')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publishers for gimbal control
        self.basic_control_pub = self.create_publisher(Float32MultiArray, 'gimbal_control_simple', 10)
        self.continuous_control_pub = self.create_publisher(Float32MultiArray, 'gimbal_control_move', 10)

        # Parameters for axis and button mapping
        self.enable_gimbal_button = self.declare_parameter('enable_gimbal', 9).value
        self.axis_pan = self.declare_parameter('axis_pan', 3).value
        self.axis_tilt = self.declare_parameter('axis_tilt', 4).value
        self.scale_pan = self.declare_parameter('scale_pan', 0.5).value
        self.scale_tilt = self.declare_parameter('scale_tilt', 0.5).value

    def joy_callback(self, joy_msg):
        # Check if gimbal control is enabled
        if joy_msg.buttons[self.enable_gimbal_button]:
            pan_value = joy_msg.axes[self.axis_pan] * self.scale_pan
            tilt_value = joy_msg.axes[self.axis_tilt] * self.scale_tilt
            
            # Create and send gimbal control message
            gimbal_msg = Float32MultiArray()
            gimbal_msg.data = [pan_value, tilt_value, 0.5, 0.5]  # Example speed and acceleration values
            #self.continuous_control_pub.publish(gimbal_msg)
            self.basic_control_pub.publish(gimbal_msg)

def main(args=None):
    rclpy.init(args=args)
    joystick_gimbal_control = JoystickGimbalControl()
    rclpy.spin(joystick_gimbal_control)
    joystick_gimbal_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()