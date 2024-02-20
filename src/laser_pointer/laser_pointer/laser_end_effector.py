#!/usr/bin/env python 3
import rclpy
import lgpio 
from rclpy.node import Node
class LazerEndEffector(Node):
    def __init__(self):
        super().__init__("laser_end_effector")
        self.get_logger().info("Lazer is ready")
        self.create_timer(0.5,self.blink_Led)
        self.h = lgpio.gpiochip_open(0)
        self.LED = 23
        lgpio.gpio_claim_output(self.h,self.LED)
        self.currentState = 0

    def blink_Led(self):
        if self.currentState == 0:
            self.currentState = 1
        elif self.currentState == 1:
            self.currentState = 0
        lgpio.gpio_write(self.h, self.LED, self.currentState )
        
def main(args=None):
    rclpy.init(args=args)
    node = LazerEndEffector()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()