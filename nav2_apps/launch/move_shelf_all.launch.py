# nav2_apps/launch/move_shelf.launch.py
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

    # Declare use_sim_time launch arg
    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        )

    app_config = os.path.join(get_package_share_directory('nav2_apps'),'config','spot-list.yaml')

    map_file_sim = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_sim.yaml')
    map_file_real = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_real.yaml')
 
    loc_config_dir = os.path.join(get_package_share_directory('localization_server'), 'config')

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')

    # File selection logic
    config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config')

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
            parameters=[os.path.join(loc_config_dir, 'amcl_config_sim.yaml')],
            condition=IfCondition(use_sim_time)
        )

    amcl_node_real = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(loc_config_dir, 'amcl_config_real.yaml')],
            condition=UnlessCondition(use_sim_time)
        )
 
    # Conditional planner server nodes
    planner_server_node_sim = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'planner_sim.yaml')],
            condition=IfCondition(use_sim_time)
        )

    planner_server_node_real = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'planner_real.yaml')],
            condition=UnlessCondition(use_sim_time)
        )

    controller_server_node_sim = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'controller_sim.yaml')],
            condition=IfCondition(use_sim_time),
            remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
        )

    controller_server_node_real = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'controller_real.yaml')],
            condition=UnlessCondition(use_sim_time)
       )

    bt_navigator_node_sim = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(config_dir, 'bt_navigator_sim.yaml')],
            condition=IfCondition(use_sim_time)
            #remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
        )

    bt_navigator_node_real = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(config_dir, 'bt_navigator_real.yaml')],
            condition=UnlessCondition(use_sim_time)
        )

    recoveries_server_node_sim = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'recoveries_sim.yaml')],
            condition=IfCondition(use_sim_time),
            remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
        )

    recoveries_server_node_real = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'recoveries_real.yaml')],
            condition=UnlessCondition(use_sim_time)
        )

    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator', 'recoveries_server']}
                        ]
        )    

    delay_duration = 3.0  # 3 seconds delay
    delay_action2 = TimerAction(
        period=delay_duration,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}])]
        )

    attach_shelf_node = Node(
            package='attach_shelf',
            executable='approach_service_server',
            name='approach_service_server',
            output='screen'
        )

    move_shelf_node = Node(
            package='nav2_apps',
            name='move_shelf_to_ship',
            executable='move_shelf_to_ship',
            output='screen',
            parameters = [{'use_sim_time': use_sim_time},
                         app_config]
        )

    return LaunchDescription([
        declare_use_sim_time,

        #map_server_node_sim,
        #map_server_node_real,
        #amcl_node_sim,
        #amcl_node_real,

        #planner_server_node_sim,
        #planner_server_node_real,
        #controller_server_node_sim,
        #controller_server_node_real,
        #bt_navigator_node_sim,
        #bt_navigator_node_real,
        #recoveries_server_node_sim,
        #recoveries_server_node_real,
        #lifecycle_manager_node,
        #delay_action2,

        attach_shelf_node,
        move_shelf_node
    ])
