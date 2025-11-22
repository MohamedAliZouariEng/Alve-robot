import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments for model path and spawn position
    models_dir = get_package_share_directory('deepmind_bot_gazebo') + '/models'
    banana_model_path = os.path.join(models_dir, 'banana', 'model.sdf')
    #hammer_model_path = os.path.join(models_dir, 'hammer', 'model.sdf')
    apple_model_path = os.path.join(models_dir, 'apple', 'model.sdf')

    bench_height = 0.42

    banana_position = [0.0, -4.7, bench_height] 
    #hammer_position = [0.0, -5.3, bench_height] 
    apple_position = [0.0, -6.0, bench_height] 

    # Spawn ROBOT Set Gazebo
    spawn_banana = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'banana', '-file', banana_model_path, '-x', str(banana_position[0]), '-y', str(banana_position[1]), '-z', str(banana_position[2])],
            output='screen'
        )
    spawn_apple = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'apple', '-file', apple_model_path, '-x', str(apple_position[0]), '-y', str(apple_position[1]), '-z', str(apple_position[2])],
            output='screen'
        )
    # create and return launch description object
    return LaunchDescription(
        [
            spawn_banana,
            spawn_apple,
        ]
    )