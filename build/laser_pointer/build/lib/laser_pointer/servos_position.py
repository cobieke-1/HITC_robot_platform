#!/usr/bin/env python 3
import rclpy
import lgpio 
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
class ServoPositionNode(Node):
    def __init__(self):
        super().__init__("servo_position")
        self.get_logger().info("servos are ready")
        self.subscriber_ = self.create_subscription(Int32MultiArray, "servo_pwms", self.callback_servo_pwms,10)
        self.h = lgpio.gpiochip_open(0)
        self.servo1 = 12
        self.servo2 = 13
        self.frequency = 1000
        lgpio.gpio_claim_output(self.h,self.servo1)
        lgpio.gpio_claim_output(self.h,self.servo2)

    def callback_servo_pwms(self, servo_positions):
        servo1percent, servo2percent =  servo_positions.data
        lgpio.tx_pwm(self.h, self.servo1, self.frequency, servo1percent)
        lgpio.tx_pwm(self.h, self.servo2, self.frequency, servo2percent)
        
def main(args=None):
    rclpy.init(args=args)
    node = ServoPositionNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()