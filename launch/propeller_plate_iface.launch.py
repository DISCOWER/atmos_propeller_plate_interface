from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch argument
    namespace = LaunchConfiguration("namespace")

    # Declare argument with default
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the propeller plate interface node",
    )

    config = os.path.join(
        get_package_share_directory("propeller_plate_iface"),
        "config",
        "parameters.yaml",
    )

    node = Node(
        package="propeller_plate_iface",
        executable="propeller_plate_iface_node",
        name="propeller_plate_iface_node",
        namespace=namespace,
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([declare_namespace, node])
