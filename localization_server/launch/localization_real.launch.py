import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_real.yaml')
 
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_real.yaml')

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':map_file} 
                       ])

    amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml])

    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}])    

    # create and return launch description object
    return LaunchDescription(
        [
             map_server_node,
             amcl_node,
             lifecycle_manager_node
        ]
    )