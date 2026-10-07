"""PhaseSpace tracker (+ RViz): ros2 launch phasespace_ros2 phasespace.launch.py [rviz:=false]"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device, profile, freq = LaunchConfiguration("device"), LaunchConfiguration("profile"), LaunchConfiguration("freq")
    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="cs-phasespace.cs.umn.edu"),
        DeclareLaunchArgument("profile", default_value="", description="session profile (empty: server default)"),
        DeclareLaunchArgument("freq", default_value="60"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="phasespace_ros2", executable="tracker", output="screen",
            arguments=["--device", device, "--freq", freq, "--stream", "udp", "--profile", profile],
        ),
        Node(
            package="rviz2", executable="rviz2", output="log", condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", PathJoinSubstitution([FindPackageShare("phasespace_ros2"), "rviz", "phasespace.rviz"])],
        ),
    ])
