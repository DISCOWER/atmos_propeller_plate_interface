from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('propeller_plate_iface'),
        'config',
        'parameters.yaml'
    )

    return LaunchDescription(
        [
            Node(
                package="propeller_plate_iface",
                executable="propeller_plate_iface_node",
                name="propeller_plate_iface_node",
                output="screen",
                parameters=[config],
            )
        ]
    )
