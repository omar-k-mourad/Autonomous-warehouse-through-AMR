import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Environment
    package_dir = get_package_share_directory('my_bot')

    # Constant parameters
    lifecycle_nodes = ['collision_monitor']
    autostart = True

    # Launch arguments
    namespace1 = LaunchConfiguration('namespace1')
    namespace2 = LaunchConfiguration('namespace2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Declare the launch arguments
    declare_namespace1_cmd = DeclareLaunchArgument(
        'namespace1',
        default_value='robot1',
        description='Namespace for robot1')

    declare_namespace2_cmd = DeclareLaunchArgument(
        'namespace2',
        default_value='robot2',
        description='Namespace for robot2')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'config', 'collision_monitor_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Create our own temporary YAML files that include substitutions
    def create_param_file(namespace):
        param_substitutions = {'use_sim_time': use_sim_time}
        return RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    configured_params1 = create_param_file(namespace1)
    configured_params2 = create_param_file(namespace2)

    # Nodes launching commands
    def create_lifecycle_manager_cmd(namespace):
        return Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]
        )

    def create_collision_monitor_cmd(namespace, configured_params):
        return Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params]
        )

    # Group actions for each robot
    collision_monitor_1 = GroupAction(
        actions=[
            create_lifecycle_manager_cmd(namespace1),
            create_collision_monitor_cmd(namespace1, configured_params1)
        ]
    )

    collision_monitor_2 = GroupAction(
        actions=[
            create_lifecycle_manager_cmd(namespace2),
            create_collision_monitor_cmd(namespace2, configured_params2)
        ]
    )

    # Launch description
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_namespace1_cmd)
    ld.add_action(declare_namespace2_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Node launching commands
    ld.add_action(collision_monitor_1)
    ld.add_action(collision_monitor_2)

    return ld
