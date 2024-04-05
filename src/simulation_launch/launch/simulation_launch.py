from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription ([
        Node(
            package='get_pedestrian_position_current',
            executable='get_pedestrian_position_current_exec',
            name='get_pedestrian_position_current_node',
            output='screen'
        ),
        Node(
            package='driving_decision',
            executable='driving_decision_exec',
            name='driving_decision_node',
            output='screen'
        ),
        Node(
            package='driving_decision',
            executable='update_vehicle_state_exec',
            name='update_vehicle_state_node',
            output='screen'
        ),
        Node(
            package='run_simulation',
            executable='run_simulation_exec',
            name='run_simulation_node',
            output='screen'
        ),

    ])