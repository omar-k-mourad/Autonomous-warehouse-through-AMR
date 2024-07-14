import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import LiftJoint

class LiftJointActionClient(Node):

    def __init__(self):
        super().__init__('lift_joint_action_client')
        self._action_client = ActionClient(self, LiftJoint, 'move_lift_joint')

    def send_goal(self, position):
        goal_msg = LiftJoint.Goal()
        goal_msg.target_position = position

        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Current position: {feedback_msg.feedback.current_position}')

def main(args=None):
    rclpy.init(args=args)
    action_client = LiftJointActionClient()
    action_client.send_goal(0.03)  # Example target position
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
