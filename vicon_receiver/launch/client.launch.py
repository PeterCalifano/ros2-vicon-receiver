"""Launch the vicon_client node with a parameters file.

The node has no built-in parameter defaults: every value comes from the parameters file, e.g.
config/vicon_client.yaml at the root of the repository. The file is therefore a required launch
argument:

    ros2 launch vicon_receiver client.launch.py params_file:=config/vicon_client.yaml

scripts/launch.sh does the same with ros2 run, after checking that the Vicon server is reachable.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Declare the required params_file argument and start the node with it."""
    # No default value: ros2 launch stops with an error if the argument is missing
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        description="Parameters file of the vicon_client node (e.g. config/vicon_client.yaml)",
    )

    vicon_client_node = Node(
        package="vicon_receiver",
        executable="vicon_client",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, vicon_client_node])
