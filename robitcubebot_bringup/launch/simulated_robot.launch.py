import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():



    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robitcubebot_description"),
            "launch",
            "gazebo.launch.py"
        ),

    )
    rviz_config_path = os.path.join(get_package_share_directory('robitcubebot_description'),
                                    'rviz', 'my_robot_config.rviz')
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robitcubebot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    moveit = IncludeLaunchDescription(
           os.path.join(
              get_package_share_directory("robitcubebot_moveit"),
               "launch",
              "moveit.launch.py"
           ),
     
       )

    
    return LaunchDescription([
        gazebo,
        controller,
        rviz2_node,
        moveit
        
    ])
