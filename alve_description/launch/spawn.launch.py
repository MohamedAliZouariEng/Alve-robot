import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, LaunchConfiguration)
from launch_ros.actions import (Node, SetParameter)
from launch_ros.parameter_descriptions import ParameterValue


# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get Package Description and Directory #
    package_description = "alve_description"
    package_directory = get_package_share_directory(package_description)

    # Load URDF File #
    urdf_file = 'alve.urdf'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print("URDF Loaded !")

    # Robot State Publisher (RSP) #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 
                     'robot_description': ParameterValue(Command(['xacro ', robot_desc_path]), value_type=str)
        }]
    )

    # Spawn the Robot #
    declare_spawn_x = DeclareLaunchArgument("x", default_value="0.0",
                                            description="Model Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0",
                                            description="Model Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.9",
                                            description="Model Spawn Z Axis Value")
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="alve_spawn",
        arguments=[
            "-name", "alve",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # Create and Return the Launch Description Object #
    return LaunchDescription(
        [
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo) #
            SetParameter(name="use_sim_time", value=True),
            robot_state_publisher_node,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            gz_spawn_entity,
        ]
    )