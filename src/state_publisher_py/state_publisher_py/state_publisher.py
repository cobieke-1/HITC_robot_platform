#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from example_interfaces.msg import Int32

class PlatformStatePublisherNode(Node):
    def __init__(self):
        super().__init__("state_publisher")
        self.get_logger().info("State_publisher node started.")
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # self.subscriber_ = self.create_subscription(Int32,'joint_angles', self.send_angles)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.joints = JointState()
        self.results = []
        with open('/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_angle_trajectory.csv', newline='') as csvfile:
                csvreader = csv.reader(csvfile, delimiter=',')
                for row in csvreader:
                    self.results.append(row)
        self.count = 0
        
        
    def timer_callback(self):
        # Update joint state
        now = self.get_clock().now()
        self.joints.header.stamp = now.to_msg()
        self.joints.name = ['base_L1', 'L1-L2','L2-EE'] 
        bl1 = float(self.results[self.count][0])
        l12 = float(self.results[self.count][1])
        l2e = float(self.results[self.count][2])
        self.joints.position = [bl1, l12, l2e]
        if (self.count < len(self.results)-1):
            self.count += 1
        self.publisher_.publish(self.joints)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformStatePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
