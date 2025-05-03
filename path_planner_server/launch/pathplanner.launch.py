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

    rviz_config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')

    # Declare use_sim_time launch arg
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # File selection logic
    config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config')

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

    lifecycle_manager_node2 = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['planner_server', 'controller_server', 'bt_navigator', 'recoveries_server']}])    

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

    # create and return launch description object
    return LaunchDescription(
        [
             declare_use_sim_time,
             planner_server_node_sim,
             planner_server_node_real,
             controller_server_node_sim,
             controller_server_node_real,
             bt_navigator_node_sim,
             bt_navigator_node_real,
             recoveries_server_node_sim,
             recoveries_server_node_real,
             lifecycle_manager_node2,
             delay_action2
        ]
    )