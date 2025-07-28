from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    line_controller = Node(
        package='vision_processors',
        executable='line_controller',
        name='line_controller',
        output='screen'
    )

    return LaunchDescription([line_controller])
