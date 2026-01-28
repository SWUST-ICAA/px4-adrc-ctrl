import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(os.getcwd(), "config", "adrc_controller.yaml")
    if not os.path.exists(params):
        params = os.path.join(get_package_share_directory("px4_adrc_ctrl"), "config", "adrc_controller.yaml")

    return LaunchDescription([
        Node(
            package="px4_adrc_ctrl",
            executable="adrc_controller_node",
            name="px4_adrc_controller",
            output="screen",
            parameters=[params],
        )
    ])
