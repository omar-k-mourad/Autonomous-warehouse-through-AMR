import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("robitcubebot_controller"), "config", "joy_config.yaml")]
    )

    return LaunchDescription(
        [
            joy_node
        ]
    )
