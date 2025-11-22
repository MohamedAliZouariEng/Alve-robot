#! /usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

   

    static_tf_pub_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_base_footprint',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0.65', '0', '0', '0', 'base_footprint', 'base_link']
    )

    
    

    static_tf_pub_4 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_base_footprint',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0.078', '1.19', '1.57', '0', '0', 'base_footprint', 'camera']
    )

    static_tf_pub_5 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_lidar_link_base_footprint',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0.1', '1.25', '0', '0', '0', 'base_footprint', 'lidar_link']
    )

    #ros2 run tf2_ros static_transform_publisher --x 0 --y 0.078 --z 1.19 --roll -1.57 --pitch 0 --yaw 0 --frame-id base_footprint --child-frame-id camera

    return LaunchDescription(
        [
            static_tf_pub_4,
            static_tf_pub_2
        ]
    )