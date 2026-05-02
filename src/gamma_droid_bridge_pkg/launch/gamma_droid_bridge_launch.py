import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_share = get_package_share_directory("gamma_droid_bridge_pkg")
    config_file = os.path.join(package_share, "config", "bridge_config.yaml")

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gamma_droid_parameter_bridge",
        output="screen",
        parameters=[{"config_file": config_file}],
    )

    return LaunchDescription([bridge_node])
