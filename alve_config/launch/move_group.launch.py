from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("alve", package_name="alve_config").to_moveit_configs()
    initial_state_file = PathJoinSubstitution(
        [get_package_share_directory("alve_config"), "config", "initial_positions.yaml"]
    )
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
            {"robot_description_planning.initial": initial_state_file},
        ],
    )

    return LaunchDescription(
        [move_group_node]
    )