#!/usr/bin/env python 3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32MultiArray
class PositionPublisher(Node):
    def __init__(self):
        super().__init__("position_publisher")
        self.get_logger().info("Position publisher started. Lets tell those joints what to do!")
        self.publisher_ = self.create_publisher(Int32MultiArray, "servo_pwms",10)
        self.timer_ = self.create_timer(1, self.callback_servo_publishing)

    def  callback_servo_publishing(self):
        percent_duty_cycle = Int32MultiArray()
        percent_duty_cycle.data = [50, 50]
        self.publisher_.publish(percent_duty_cycle)
def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()