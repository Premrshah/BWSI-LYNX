from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    front_camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='front_cam',
        parameters=[
            {'camera':0},
            {'width':800},
            {'height':600}
        ],
        output='screen'
    )

    down_camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='down_cam',
        parameters=[
            {'camera':1},
            {'width':800},
            {'height':600}
        ],
        output='screen'
    )

    return LaunchDescription([front_camera, down_camera])