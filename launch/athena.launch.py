from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',
            name='moveit_joystick_control_spawner',
            output='screen',
            arguments=[
                'moveit_twist_controller',  # Controller name (positional)
                '-c', '/controller_manager' # Points to the controller manager node
            ]
        )
    ])