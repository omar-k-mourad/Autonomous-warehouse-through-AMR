import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv


global robot_pose_dic
robot_pose_dic = {'robot1' : (0,0), 'robot2' : (0,0)}

class MinimalSubscriber(Node):

    robot_pose_dic = {'robot1' : (0,0), 'robot2' : (0,0)}
    
    def __init__(self, robot_names):
        super().__init__('minimal_subscriber')
        for robot_name in robot_names:  
            self.subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                f'{robot_name}/amcl_pose',
                lambda msg,
                robot_name=robot_name: self.listener_callback(msg,robot_name),
                10)
            self.get_logger().info(f'i Subscribed to topic {self.subscription.topic_name}')
            self.subscription  # prevent unused variable warning

    def listener_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'I heard from {robot_name}: "%s"' % msg.pose.pose.position.x)
        self.robot_pose_dic[robot_name] = (x,y)
        print(f'\n{self.robot_pose_dic}')
        self.write_robot_pose_to_text(self.robot_pose_dic)

    def write_robot_pose_to_text(self, robot_pose_dic):
        print("here 1")
        with open("pose.csv", "w") as text_file:
            for robot_name, pose in robot_pose_dic.items():
                print(f"{robot_name}, {pose}")
                text_file.write(f"{robot_name}: {pose}\n")

    def stop(self):
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    print("1")
    robot_names = {'robot1', 'robot2'}
    minimal_subscriber = MinimalSubscriber(robot_names = robot_names)
  
    print("2")
    rclpy.spin(minimal_subscriber)
    print("3")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
   

if __name__ == '__main__':
    main()
