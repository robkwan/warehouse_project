import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare use_sim_time launch arg
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    spots_config = os.path.join(get_package_share_directory('nav2_apps'),'config','spot-list.yaml')

    attach_shelf_node = Node(
        package='attach_shelf',
        executable='approach_service_server',
        name='approach_service_server',
        output='screen',
        parameters = [{'use_sim_time':use_sim_time}]
    )

    move_shelf_node = Node(
        package = 'nav2_apps',
        name = 'move_shelf_to_ship',
        executable = 'move_shelf_to_ship',
        parameters = [{'use_sim_time':use_sim_time},
                        spots_config],
        output = 'screen',
    )
   
    return LaunchDescription([ 
    
        declare_use_sim_time,

        attach_shelf_node,

        #move_shelf_node

    ])