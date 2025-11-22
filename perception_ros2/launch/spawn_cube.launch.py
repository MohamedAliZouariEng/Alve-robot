import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments for model path and spawn position
    models_dir = get_package_share_directory('perception_ros2') + '/models'
    cube_model_path = os.path.join(models_dir, 'demo_cube', 'model.sdf')
    position = [0.0, -5.0, 1.2] # bench orange side
    # position = [0.0, 0.7, 1.2] # middle bench

    # Spawn ROBOT Set Gazebo
    spawn_cube = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'cube', '-file', cube_model_path, '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2])],
            output='screen'
        )

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_cube,
        ]
    )