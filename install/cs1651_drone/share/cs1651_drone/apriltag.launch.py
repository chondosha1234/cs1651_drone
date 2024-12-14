import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # get path to share directory for apriltag_ros
    apriltag_ros_share_dir = get_package_share_directory('apriltag_ros')
    
    # get path to tags yaml file
    tags_36h11_yaml_file = os.path.join(apriltag_ros_share_dir, 'cfg', 'tags_36h11.yaml')

    launch_description = LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[],
            remappings=[
               # ('/image_raw', '/camera/image_raw'),
               # ('/camera_info', '/camera/camera_info')
            ]
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                #('/image_raw', '/camera/image_raw'),
                #('/camera_info', '/camera/camera_info')
                ('image_rect', '/image_raw'),
                ('camera_info', '/camera_info')
            ],
            parameters=[
                {'params_file': tags_36h11_yaml_file}
            ]
        ),
        Node(
            package='cs1651_drone',
            executable='camera_node',
            output='screen',
        )
    ])

    return launch_description


