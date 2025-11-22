import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments for model path and spawn position
    models_dir = get_package_share_directory('deepmind_bot_gazebo') + '/models'
    local_models_dir = get_package_share_directory('perception_ros2') + '/models'


    apple_model_path = os.path.join(local_models_dir, 'apple', 'model.sdf')
    banana_model_path = os.path.join(local_models_dir, 'banana', 'model.sdf')
    tennis_ball_model_path = os.path.join(local_models_dir, 'tennis_ball', 'model.sdf')

    bench_height = 0.42

    apple1_position = [0.0, -6.3, bench_height] 
    apple2_position = [-0.2, -6.1, bench_height] 
    apple3_position = [-0.08, -6.03, bench_height] 
    apple4_position = [0.15, -5.9, bench_height] 
    apple5_position = [0.1, -5.6, bench_height] 

    banana1_position = [-0.10, -5.6, bench_height, 0.4] 
    banana2_position = [-0.06, -5.9, bench_height, -0.6] 
    banana3_position = [-0.2, -5.8, bench_height, 1.4]  

    tennis_ball1_position = [-0.1, -5.4, bench_height+0.03, 0.2] 
    tennis_ball2_position = [-0.15, -5.2, bench_height+0.03, 0.2] 


    # Spawn ROBOT Set Gazebo
    spawn_apple1 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'apple1', '-file', apple_model_path, '-x', str(apple1_position[0]), '-y', str(apple1_position[1]), '-z', str(apple1_position[2])],
            output='screen'
        )
    spawn_apple2 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'apple2', '-file', apple_model_path, '-x', str(apple2_position[0]), '-y', str(apple2_position[1]), '-z', str(apple2_position[2])],
            output='screen'
        )
    spawn_apple3 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'apple3', '-file', apple_model_path, '-x', str(apple3_position[0]), '-y', str(apple3_position[1]), '-z', str(apple3_position[2])],
            output='screen'
        )
    spawn_apple4 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'apple4', '-file', apple_model_path, '-x', str(apple4_position[0]), '-y', str(apple4_position[1]), '-z', str(apple4_position[2])],
            output='screen'
        )
    spawn_apple5 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'apple5', '-file', apple_model_path, '-x', str(apple5_position[0]), '-y', str(apple5_position[1]), '-z', str(apple5_position[2])],
            output='screen'
        )
    spawn_banana1 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'banana1', '-file', banana_model_path, '-x', str(banana1_position[0]), '-y', str(banana1_position[1]), '-z', str(banana1_position[2]), '-Y', str(banana1_position[3])],
            output='screen'
        )
    spawn_banana2 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'banana2', '-file', banana_model_path, '-x', str(banana2_position[0]), '-y', str(banana2_position[1]), '-z', str(banana2_position[2]), '-Y', str(banana2_position[3])],
            output='screen'
        )
    spawn_banana3 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'banana3', '-file', banana_model_path, '-x', str(banana3_position[0]), '-y', str(banana3_position[1]), '-z', str(banana3_position[2]), '-Y', str(banana3_position[3])],
            output='screen'
        )
    spawn_tennis_ball1 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'tennis_ball1', '-file', tennis_ball_model_path, '-x', str(tennis_ball1_position[0]), '-y', str(tennis_ball1_position[1]), '-z', str(tennis_ball1_position[2]), '-Y', str(tennis_ball1_position[3])],
            output='screen'
        )
    spawn_tennis_ball2 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'tennis_ball2', '-file', tennis_ball_model_path, '-x', str(tennis_ball2_position[0]), '-y', str(tennis_ball2_position[1]), '-z', str(tennis_ball2_position[2]), '-Y', str(tennis_ball2_position[3])],
            output='screen'
        )
    # create and return launch description object
    return LaunchDescription(
        [
            spawn_apple1,
            spawn_apple2,
            spawn_apple3,
            spawn_apple4,
            spawn_apple5,
            spawn_banana1,
            spawn_banana2,
            spawn_banana3,
            spawn_tennis_ball1,
            spawn_tennis_ball2,
        ]
    )

    #arguments=['-entity', 'cube', '-file', cube_model_path, '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]), '-R', 'rotation_radians_x', '-P', 'rotation_radians_y', '-Y', 'rotation_radians_z']
