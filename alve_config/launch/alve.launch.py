from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.actions import TimerAction

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)



    # Get Package Description and Directory #
    package_description = "alve_description"
    package_directory = get_package_share_directory(package_description)

    # Load URDF File #
    urdf_file = 'alve.urdf'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print("URDF Loaded !")

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
    robot_description = {'robot_description': robot_description_content}
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
        parameters=[{'use_sim_time': True, 
                     'robot_description': ParameterValue(Command(['xacro ', robot_desc_path]), value_type=str)
        }]
    )

    depth_node = Node(
        package='depth',
        executable='depth_node',
        name='depth_node',
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'alve', '-allow_renaming', 'true',
                   "-x", "0.0",  
                   "-y", "0.0", 
                   "-z", "0.8",
                   "-R", "0.0",
                   "-P", "0.0",
                   "-Y", "-1.6"],
    )

    start_arm_right_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm__right_controller'],
        output='screen')
    
    start_arm_left_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_left_controller'],
        output='screen')
    
    start_both_arms_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'both_arms_controller'],
        output='screen')
    
    start_hand_right_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'hand_right_controller'],
        output='screen')
    
    start_hand_left_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'hand_left_controller'],
        output='screen')
    
    start_neck_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'neck_controller'],
        output='screen')
    
    start_mecanum_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mecanum_drive_controller'],
        output='screen')
    
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')
    
    delayed_start = TimerAction(
        period=15.0,
        #period=40.0, minimize this value (computer performance HIGH) or maximize it (computer performance LOW)
        actions=[start_joint_state_broadcaster_cmd]
    )


    # Register event handlers for sequencing
    # Launch the joint state broadcaster after spawning the robot
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_both_arms_controller_cmd]))
    
    load_both_arms_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_both_arms_controller_cmd,
            on_exit=[start_hand_right_controller_cmd]))
    
    load_hand_right_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_hand_right_controller_cmd,
            on_exit=[start_hand_left_controller_cmd]))
    
    load_hand_left_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_hand_left_controller_cmd,
            on_exit=[start_neck_controller_cmd]))
    
    load_neck_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_neck_controller_cmd,
            on_exit=[start_mecanum_drive_controller_cmd]))

    



    remappings = [('/camera', '/camera/image'),('/camera_two', '/camera_two/image'),
                  
                  ('/camera_info', '/camera/camera_info')]

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                    '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',

                    '/camera_two/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera_two/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',

                    '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '/mecanum_drive_controller/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist'
                    ],
        output='screen',
        remappings=remappings,
    )

    

    ld = LaunchDescription()

    launch_ros_gz_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 /home/ubuntu24/ros2_ws/src/alve_description/worlds/alve_warehouse.sdf'])])

    
    use_sim_time_declared = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')
    
    ld.add_action(launch_ros_gz_world)
    ld.add_action(gz_spawn_entity)
    ld.add_action(delayed_start)
    ld.add_action(load_joint_state_broadcaster_cmd)
    ld.add_action(load_both_arms_controller_cmd)
    ld.add_action(load_hand_left_controller_cmd)
    ld.add_action(load_hand_right_controller_cmd)
    ld.add_action(load_neck_controller_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(bridge)
    ld.add_action(use_sim_time_declared)
    ld.add_action(depth_node)


    return ld
