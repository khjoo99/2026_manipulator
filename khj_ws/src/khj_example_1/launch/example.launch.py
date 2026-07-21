from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="khj_example_1", executable="m_pub"),
            Node(package="khj_example_1", executable="m_sub"),
            Node(package="khj_example_1", executable="m2_sub"),
            Node(package="khj_example_1", executable="t_pub"),
            Node(package="khj_example_1", executable="mt_sub"),
        ]
    )
