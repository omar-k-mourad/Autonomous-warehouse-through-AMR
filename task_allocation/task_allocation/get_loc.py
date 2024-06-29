import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import argparse

class MinimalSubscriber(Node):

    def __init__(self, robot_names):
        super().__init__('minimal_subscriber')
        self.robot_pose_dic = {robot_name: (0, 0) for robot_name in robot_names}
        for robot_name in robot_names:
            self.subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                f'{robot_name}/amcl_pose',
                lambda msg, robot_name=robot_name: self.listener_callback(msg, robot_name),
                10)
            self.get_logger().info(f'Subscribed to topic {self.subscription.topic_name}')
            self.subscription  # prevent unused variable warning

    def listener_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'I heard from {robot_name}: "{x}"')
        self.robot_pose_dic[robot_name] = (x, y)
        print(f'\n{self.robot_pose_dic}')
        self.write_robot_pose_to_text(self.robot_pose_dic)

    def write_robot_pose_to_text(self, robot_pose_dic):
        print("Writing to CSV")
        with open("pose.csv", "w") as text_file:
            for robot_name, pose in robot_pose_dic.items():
                print(f"{robot_name}, {pose}")
                text_file.write(f"{robot_name}: {pose}\n")

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Minimal Subscriber')
    parser.add_argument('--robot_names', nargs='*', default=[""], help='List of robot names to subscribe to')
    args = parser.parse_args()

    robot_names = args.robot_names
    minimal_subscriber = MinimalSubscriber(robot_names=robot_names)
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
