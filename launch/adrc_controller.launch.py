import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("px4_adrc_ctrl")
    installed_params = os.path.join(pkg_share, "config", "adrc_controller.yaml")
    # Convenience: if you run `ros2 launch` from the package root, prefer the workspace file so
    # edits to `config/adrc_controller.yaml` take effect immediately without rebuild/install.
    cwd_params = os.path.join(os.getcwd(), "config", "adrc_controller.yaml")
    default_params = cwd_params if os.path.exists(cwd_params) else installed_params

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the controller YAML parameters file.",
            ),
            Node(
                package="px4_adrc_ctrl",
                executable="adrc_controller_node",
                name="px4_adrc_controller",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            )
        ]
    )
