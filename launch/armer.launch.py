from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    cfg_path = PathJoinSubstitution(
        [FindPackageShare("armer"), "cfg/panda_sim.yaml"]
    )

    armer_node = Node(
        package='armer',
        namespace='armer',
        executable='armer',
        name='armer',
        output='screen',
        parameters=[{"config": cfg_path}]
    )

    ld.add_action(armer_node)

    return ld