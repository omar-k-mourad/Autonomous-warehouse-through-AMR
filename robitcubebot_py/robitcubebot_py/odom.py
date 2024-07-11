import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class OdomRepublisher(Node):

    def __init__(self):
        super().__init__('odom_republisher')
        self.subscription = self.create_subscription(
            Odometry,
            '/robitcubebot_controller/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(
            Odometry,
            '/odom',
            10)
        self.get_logger().info('OdomRepublisher node is running.')

    def odom_callback(self, msg):
        # Create an Odometry message
        stamped_msg = Odometry()
        stamped_msg.header = Header()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()  # Set current time
        stamped_msg.header.frame_id = msg.header.frame_id
        stamped_msg.child_frame_id = msg.child_frame_id
        stamped_msg.pose = msg.pose
        stamped_msg.twist = msg.twist

        # Publish the Odometry message
        self.publisher.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    odom_republisher = OdomRepublisher()
    rclpy.spin(odom_republisher)
    odom_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
