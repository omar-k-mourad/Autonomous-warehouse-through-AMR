import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.duration import Duration

from control_msgs.action import LiftJoint
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class LiftJointActionServer(Node):

    def __init__(self):
        super().__init__('lift_joint_action_server')
        self._action_server = ActionServer(
            self,
            LiftJoint,
            'move_lift_joint',
            self.execute_callback)
        
        self.publisher = self.create_publisher(JointTrajectory, '/lift_joint_controller/joint_trajectory', 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.current_position = None

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'lift_joint':
                self.current_position = msg.position[i]

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        target_position = goal_handle.request.target_position
        
        if target_position < -0.05 or target_position > 0.035:
            self.get_logger().error('Target position is out of bounds')
            goal_handle.succeed()
            result = LiftJoint.Result()
            result.success = False
            result.current_position = self.current_position
            return result

        self.move_to_position(target_position)

        while not self.is_position_reached(target_position):
            self.get_logger().info(f'Current position: {self.current_position}')
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle.succeed()
        result = LiftJoint.Result()
        result.success = True
        result.current_position = self.current_position
        return result

    def get_current_position(self):
        return self.current_position

    def move_to_position(self, target_position):
        msg = JointTrajectory()
        msg.joint_names = ['lift_joint']
        point = JointTrajectoryPoint()
        point.positions = [target_position]
        point.time_from_start = Duration(seconds=5.0).to_msg()
        msg.points = [point]
        self.publisher.publish(msg)

    def is_position_reached(self, target_position):
        if self.current_position is None:
            return False
        return abs(self.current_position - target_position) < 0.01  # Example tolerance value

def main(args=None):
    rclpy.init(args=args)
    lift_joint_action_server = LiftJointActionServer()
    rclpy.spin(lift_joint_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
