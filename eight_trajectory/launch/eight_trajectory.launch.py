from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the wheel velocity publisher
        Node(
            package='eight_trajectory',
            executable='eight_trajectory',
            name='eight_trajectory',
            output='screen'
        ),

        # Start the kinematic model converter
        Node(
            package='kinematic_model',
            executable='kinematic_model_node',
            name='kinematic_model',
            output='screen'
        )
    ])
