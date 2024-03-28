#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

class PlatformStatePublisherNode(Node):
    def __init__(self):
        super().__init__("state_publisher")
        self.get_logger().info("State_publisher node started.")
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.publisher_micro_ros = self.create_publisher(Float32MultiArray, '/trajectory_for_esp32',10)
        # self.subscriber_ = self.create_subscription(Float32MultiArray,'/micro_ros_arduino_node_publisher', self.subscriber_send_angles,10)
        self.timer = self.create_timer(0.01, self.timer_publish_angles_to_esp)

        
        self.results = []
        # When node starts the trajectory must already be in the working directory.  We will then put csv trajectory into an array and send each waypoint to the robot later.
        with open('/home/chris/capstone/hitc_ws/src/state_publisher_py/state_publisher_py/joint_angle_trajectory.csv', newline='') as csvfile:
                csvreader = csv.reader(csvfile, delimiter=',')
                for row in csvreader:
                    self.results.append(row)

        self.waypoint_index = 0 # Current waypoint index.
        # just test variables below.
        self.test_publish = Float32MultiArray()
        self.test_publish.data = [0.01, 0.45]
        
        
    def timer_publish_angles_to_esp(self): #should send jointangles to esp32 (currently still sends directly to robot simulation)
        # Update joint state
        joints = JointState()
        now = self.get_clock().now()
        joints.header.stamp = now.to_msg()
        joints.name = ['base_L1', 'L1-L2','L2-EE'] 
        bl1 = float((self.results[self.waypoint_index][0]))
        l12 = float((self.results[self.waypoint_index][1]))
        l2e = float((self.results[self.waypoint_index][2]))

        motor_joints = Float32MultiArray()
        motor_joints.data = [bl1,l12] # the last joint is not actuated so no need to send it.
    
        self.publisher_micro_ros.publish(motor_joints)

# should comment out code below after you confirm joints have been sent to esp32 correctly.
        joints.position = [bl1,l12,l2e]
        if (self.waypoint_index < len(self.results)-1):
            self.waypoint_index += 1
        self.publisher_.publish(joints)

     
    '''
    def subscriber_send_angles(self, msg): # after recieving the encoder's angles then send these angles to the robot simulation (subscriber callback)
       # Configuring joint message to be sent
        # Update joint state
        joints = JointState()
        now = self.get_clock().now()
        joints.header.stamp = now.to_msg()
        joints.name = ['base_L1', 'L1-L2','L2-EE'] 
        bl1 = float((self.results[self.waypoint_index][0]))
        l12 = float((self.results[self.waypoint_index][1]))
        l2e = float((self.results[self.waypoint_index][2]))

        motor_joints = Float32MultiArray()
        motor_joints.data = [bl1,l12]
    
        self.publisher_micro_ros.publish(motor_joints)

        joints.position = [bl1,l12,l2e]
        if (self.waypoint_index < len(self.results)-1):
            self.waypoint_index += 1
        self.publisher_.publish(joints)
    '''
    
def main(args=None):
    rclpy.init(args=args)
    node = PlatformStatePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
