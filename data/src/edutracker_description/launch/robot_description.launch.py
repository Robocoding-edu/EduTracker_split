from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_name = "edutracker_description"

    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [FindPackageShare(pkg_name), "urdf", "robot.urdf.xacro"]
            ),
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    return LaunchDescription([robot_state_publisher])
