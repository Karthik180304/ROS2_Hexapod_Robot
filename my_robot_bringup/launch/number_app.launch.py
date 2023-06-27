from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    #remap_number_nodes = ("number", "new_number")

    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_publisher_",
        # name="new_number_publisher",
        # remappings=[
        #     ("number", "new_number")
        # ],
        # parameters=[
        #     {"number_to_publish": 4},
        #     {"publish_frequency": 0.5}
        # ]

    )

    number_counter_node = Node(
        package="my_py_pkg",
        executable="number_counter",
        # name="new_number_counter",
        # remappings=[
        #     ("number", "new_number"),
        #     ("number_count", "new_number_count")
        # ]
    )

    # led_panel_node = Node(
    #     package="battery_status",
    #     executable="led_panel"
    # )

    # battery_node = Node(
    #     package="battery_status",
    #     executable="battery"
    # )

    # countdown_server_node = Node(
    #     package="my_py_pkg",
    #     executable="countdown_action_server"
    # )

    # countdown_client_node = Node(
    #     package="my_py_pkg",
    #     executable="countdown_action_client"
    # )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    # ld.add_action(led_panel_node)
    # ld.add_action(battery_node)
    # ld.add_action(countdown_server_node)
    # ld.add_action(countdown_client_node)
    return ld