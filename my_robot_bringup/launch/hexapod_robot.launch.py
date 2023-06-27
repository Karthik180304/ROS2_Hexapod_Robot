from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    builderx_node = Node(
        package="hexapod_robot",
        executable="builderx",
        output="screen"
    )
    ld.add_action(builderx_node)

    leg_movement_node = Node(
        package="hexapod_robot",
        executable="leg_movement",
        output="screen"
    )
    ld.add_action(leg_movement_node)

    leg_movement_demo_node = Node(
        package="hexapod_robot",
        executable="leg_movement_demo",
        output="screen"
    )
    ld.add_action(leg_movement_demo_node)

    return ld
