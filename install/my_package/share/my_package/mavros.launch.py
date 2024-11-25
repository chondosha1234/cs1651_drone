from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    #config_file = os.path.join(
     #       get_package_share_directory('my_package'),
      #      'config',
       #     'px4_config.yaml'
        #    )

    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            #parameters=[config_file],
            )
        ])
