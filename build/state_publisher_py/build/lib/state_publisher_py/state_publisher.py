#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

class PlatformStatePublisherNode(Node):
    def __init__(self):
        super().__init__("state_publisher")
        self.get_logger().info("State_publisher node started.")
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.joints = JointState()
        self.count = 0.0
        
    def timer_callback(self):
        self.count += 0.01
        c = self.count 
        # Update joint state
        now = self.get_clock().now()
        self.joints.header.stamp = now.to_msg()
        self.joints.name = ['base_L1', 'L1-L2']
        self.joints.position = [c, c]
        self.publisher_.publish(self.joints)

        

        

    
def main(args=None):
    rclpy.init(args=args)
    node = PlatformStatePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
