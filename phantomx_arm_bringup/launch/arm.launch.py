from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            str(get_package_share_path('phantomx_arm_moveit_config') / 'launch' / 'rsp.launch.py')
    ]))

    robot_controllers = str(get_package_share_path('phantomx_arm_moveit_config') / 'config' / 'ros2_controllers.yaml')

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            str(get_package_share_path('phantomx_arm_moveit_config') / 'launch' / 'spawn_controllers.launch.py')
    ]))

    nodes = [
        robot_state_publisher,
        control_node,
        spawn_controllers,
    ]

    return LaunchDescription(nodes)
