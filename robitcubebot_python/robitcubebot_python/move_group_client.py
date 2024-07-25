import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from moveit_msgs.msg import Constraints, JointConstraint, RobotState


class MoveGroupClient(Node):

    def __init__(self):
        super().__init__('move_group_client')
        self._action_client = ActionClient(self, MoveGroup, 'move_group')

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Sending goal...')

        goal_msg = MoveGroup.Goal()

        goal_msg.request.group_name = 'gripper'

        start_state = RobotState()
        joint_state = JointState()
        joint_state.name = ['lift_joint']
        joint_state.position = [-0.035]
        start_state.joint_state = joint_state
        goal_msg.request.start_state = start_state

        goal_constraints = Constraints()
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'lift_joint'
        joint_constraint.position = -0.01
        joint_constraint.weight = 1.0
        goal_constraints.joint_constraints = [joint_constraint]
        goal_msg.request.goal_constraints = [goal_constraints]

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveGroupClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
