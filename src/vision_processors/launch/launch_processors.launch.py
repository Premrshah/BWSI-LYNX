from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ar_detector = Node(
        package='vision_processors',
        executable='ar_detector',
        name='ar_detector',
        output='screen'
    )

    return LaunchDescription([ar_detector])
