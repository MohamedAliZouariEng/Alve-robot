import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("name", package_name="alve_config").to_moveit_configs()
    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="pick_place_main_good",
        package="moveit2_scripts",
        executable="pick_place_good",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )
    return LaunchDescription(
        [moveit_cpp_node]
)