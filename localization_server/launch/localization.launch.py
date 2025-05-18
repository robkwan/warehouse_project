import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_dir_sim = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localizer_rviz_config.rviz')
    rviz_config_dir_real = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localizer_rviz_config_real.rviz')

    # Declare use_sim_time launch arg
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # File selection logic
    map_config_dir = os.path.join(get_package_share_directory('map_server'), 'config')
    map_file_sim = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_sim.yaml')
    map_file_real = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_real.yaml')

    config_dir = os.path.join(get_package_share_directory('localization_server'), 'config')

    map_server_node_sim = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename':map_file_sim}],
            condition=IfCondition(use_sim_time)
        )

    map_server_node_real = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename':map_file_real}],
            condition=UnlessCondition(use_sim_time)
        )

    amcl_node_sim = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(config_dir, 'amcl_config_sim.yaml')],
            condition=IfCondition(use_sim_time)
        )

    amcl_node_real = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(config_dir, 'amcl_config_real.yaml')],
            condition=UnlessCondition(use_sim_time)
        )

    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}])    

    delay_duration = 3.0  # 3 seconds delay
    delay_action_sim = TimerAction(
        period=delay_duration,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir_sim],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_sim_time)
        )]
    )

    delay_action_real = TimerAction(
        period=delay_duration,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir_real],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_sim_time)
        )]
    )

    # create and return launch description object
    return LaunchDescription(
        [
             declare_use_sim_time,
             map_server_node_sim,
             map_server_node_real,
             amcl_node_sim,
             amcl_node_real,
             lifecycle_manager_node,
             delay_action_sim,
             delay_action_real
        ]
    )