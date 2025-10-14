from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            str(get_package_share_path('phantomx_arm_bringup') / 'launch' / 'arm.launch.py')
    ]))

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            str(get_package_share_path('phantomx_arm_moveit_config') / 'launch' / 'move_group.launch.py')
    ]))

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            str(get_package_share_path('phantomx_arm_moveit_config') / 'launch' / 'moveit_rviz.launch.py')
    ]))

    nodes = [
        arm,
        move_group,
        rviz,
    ]

    return LaunchDescription(nodes)
