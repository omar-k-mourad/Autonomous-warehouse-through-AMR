import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class LiftJointPublisher(Node):
    def __init__(self):
        super().__init__('lift_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lift_joint_position = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['lift_joint']
        msg.position = [self.lift_joint_position]
        self.publisher_.publish(msg)
        self.lift_joint_position += 0.01  # Increment the position for demonstration

def main(args=None):
    rclpy.init(args=args)
    lift_joint_publisher = LiftJointPublisher()

    rclpy.spin(lift_joint_publisher)

    lift_joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
