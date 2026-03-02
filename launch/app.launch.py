from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    app_node = Node(
        package="pm_robot_dashboard",
        executable="pm_robot_dashboard",
        name="pm_robot_dashboard",
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(app_node)

    return ld
