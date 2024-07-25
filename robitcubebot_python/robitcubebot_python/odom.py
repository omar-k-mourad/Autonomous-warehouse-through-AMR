import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomRepublisher(Node):

    def __init__(self):
        super().__init__('odom_republisher')
        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(
            Odometry,
            '/odom',
            10)
        self.get_logger().info('OdomRepublisher node is running.')

    def odom_callback(self, msg):
        # Directly republish the received Odometry message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    odom_republisher = OdomRepublisher()
    rclpy.spin(odom_republisher)
    odom_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
