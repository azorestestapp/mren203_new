#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorTestNode(Node):
    def _init_(self):
        super()._init_('motor_test_node')

        #open serial port to arduino
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        self.timer = self.create_timer(0.1, self.send_command)

        self.pub = self

        self.get_logger().info('Sending constant motor command')

    def send_command(self):
        cmd = "150,150\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent: {cmd.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()

        
            