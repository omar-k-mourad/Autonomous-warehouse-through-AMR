import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class RobotLayerController(Node):
    def __init__(self, service_name):
        super().__init__('gradient_layer_controller')
        self.set_client = self.create_client(SetParameters, service_name)
        
        while not self.set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set service not available, waiting again...')
            
    def set_layer_enabled(self, param_name, enabled: bool):
        param = Parameter()
        param.name = param_name
        param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled)

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully set parameter to {enabled}')
        else:
            self.get_logger().error('Failed to set parameter')

def costmap_controller(local_robot_layer_controller, global_robot_layer_controller, enabled):
    local_robot_layer_controller.set_layer_enabled('voxel_layer.enabled', enabled)
    local_robot_layer_controller.set_layer_enabled('gradient_layer.enabled', enabled)
    global_robot_layer_controller.set_layer_enabled('obstacle_layer.enabled', enabled)
    global_robot_layer_controller.set_layer_enabled('gradient_layer.enabled', enabled)
