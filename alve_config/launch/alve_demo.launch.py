from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit

def launch_sequence_callback(context):
    """Callback function that returns the sequence of actions to execute"""
    from launch.actions import ExecuteProcess
    
    actions = []
    
    # Launch move_group
    move_group = ExecuteProcess(
        cmd=['ros2', 'launch', 'alve_config', 'move_group.launch.py'],
        output='screen',
        name='move_group_launch'
    )
    actions.append(move_group)

    # Launch blob_point_pub node
    blob_point_pub = ExecuteProcess(
        cmd=['ros2', 'run', 'blob_tracking', 'blob_point_pub'],
        output='screen',
        name='blob_point_pub'
    )
    actions.append(blob_point_pub)
    
    # Launch blob_tracker node
    blob_tracker = ExecuteProcess(
        cmd=['ros2', 'run', 'blob_tracking', 'blob_tracker'],
        output='screen',
        name='blob_tracker'
    )
    actions.append(blob_tracker)

    # Launch follow_red_line node
    follow_red_line = ExecuteProcess(
        cmd=['ros2', 'run', 'line_following', 'follow_red_line'],
        output='screen',
        name='follow_red_line'
    )
    actions.append(follow_red_line)
    
    # Launch basic_grasping after delay using bash sleep
    grasping_node = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 20 && ros2 run simple_grasping basic_grasping_perception_node --ros-args -p debug_topics:=true'],
        output='screen',
        name='basic_grasping_delayed'
    )
    actions.append(grasping_node)
    
    # Launch tf_alve after another delay
    tf_alve = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 30 && ros2 launch alve_config tf_alve.launch.py'],
        output='screen',
        name='tf_alve_delayed'
    )
    actions.append(tf_alve)
    
    # Launch pick_place after final delay
    pick_place = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 40 && ros2 launch moveit2_scripts pick_place_good.launch.py'],
        output='screen',
        name='pick_place_delayed'
    )
    actions.append(pick_place)
    
    return actions

def generate_launch_description():
    ld = LaunchDescription()
    
    # 1. First launch yolo_object_detection
    yolo_node = ExecuteProcess(
        cmd=['ros2', 'run', 'advanced_perception', 'yolo_object_detection'],
        output='screen',
        name='yolo_object_detection'
    )
    ld.add_action(yolo_node)
    
    # 2. Register the sequence to run after yolo exits
    sequence_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=yolo_node,
            on_exit=[OpaqueFunction(function=launch_sequence_callback)]
        )
    )
    ld.add_action(sequence_handler)
    
    return ld