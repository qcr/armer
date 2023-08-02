from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare a command line input argument to parameter 'config'
    DeclareLaunchArgument(
        "config", default_value=[FindPackageShare("armer"), "/cfg/panda_sim.yaml"]
    )

    # Launch armer node with config param
    return LaunchDescription([
        Node(
            package='armer',
            executable='entry_point',
            name='armer_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'config': LaunchConfiguration('config')}
            ]
        )
    ])