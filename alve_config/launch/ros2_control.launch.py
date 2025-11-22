from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('alve_description'),
                 'urdf', 'alve.urdf']
            ),
        ]
    )
    robot_description_config = xacro.process_file(robot_description_content)
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('alve_config'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': ParameterValue(robot_description_content, value_type=str)}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_right_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_right_controller'],
        output='screen',
    )

    hand_right_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_right_controller'],
        output='screen',
    )

    arm_left_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_left_controller'],
        output='screen',
    )

    hand_left_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_left_controller'],
        output='screen',
    )

    neck_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['neck_controller'],
        output='screen',
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, robot_controllers],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),
        node_robot_state_publisher,
        joint_state_publisher_node,
        
        # Controller spawners
        joint_state_broadcaster_spawner,
        arm_right_controller_spawner,
        hand_right_controller_spawner,
        arm_left_controller_spawner,
        hand_left_controller_spawner,
        neck_controller_spawner,
    ])