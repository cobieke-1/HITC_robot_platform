#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Bool
class CheckForHumanServerNode(Node):
    def __init__(self):
        super().__init__("human_checker")
        self.get_logger().info("Watching for people who are too close")
        self.publisher_ = self.create_publisher(Bool, "laser_on", 10)
        self.create_timer(1,self.check_for_face)
        self.currState = Bool()
        self.currState.data = True
    
    def check_for_face(self):
        # code to identify person
        self.currState.data = not (self.currState.data) #will replace this with the machine learning code
        self.publisher_.publish(self.currState)
def main(args=None):
    rclpy.init(args=args)
    node = CheckForHumanServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
