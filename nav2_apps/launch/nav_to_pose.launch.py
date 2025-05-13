import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
def generate_launch_description():
    
    spots_config = os.path.join(get_package_share_directory('nav2_apps'),'config','spot-list.yaml')

    attach_shelf_node = Node(
            package='attach_shelf',
            executable='approach_service_server',
            name='approach_service_server',
            output='screen'
        )
   
    return LaunchDescription([ 

    attach_shelf_node,

    Node(
        package = 'nav2_apps',
        name = 'nav_to_pose',
        executable = 'nav_to_pose',
        parameters = [{'use_sim_time':False},
                        spots_config],
        output = 'screen',
    )
    ])