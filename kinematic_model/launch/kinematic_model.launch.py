from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the wheel velocity publisher
        Node(
            package='wheel_velocities_publisher',
            executable='wheel_velocities_publisher',
            name='wheel_velocities_publisher',
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
