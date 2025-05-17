import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare use_sim_time launch arg
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'rviz', 'map_display.rviz')

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')

    sim_configuration_basename = 'cartographer_sim.lua'
    real_configuration_basename = 'cartographer_real.lua'
 
    cartographer_node_sim = Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', sim_configuration_basename],
            condition=IfCondition(use_sim_time)
    )

    cartographer_node_real = Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', real_configuration_basename],
            condition=UnlessCondition(use_sim_time)
    )

    occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.01', '-publish_period_sec', '1.0']    
    )

    delay_duration = 3.0  # 3 seconds delay
    delay_action = TimerAction(
        period=delay_duration,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}])]
    )

    # create and return launch description object
    return LaunchDescription(
        [
             declare_use_sim_time,
             cartographer_node_sim,
             cartographer_node_real,
             occupancy_grid_node,
             delay_action
        ]
    )