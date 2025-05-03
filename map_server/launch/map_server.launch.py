import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
#from launch.actions import IncludeLaunchDescription
#from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'rviz', 'map_display.rviz')

    declare_map_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Map YAML filename only (located in map_server/config/).'
    )

    # This is the LaunchConfiguration substitution for the passed-in map file name
    map_file_name = LaunchConfiguration('map_file')

    # Use PathJoinSubstitution to construct full path dynamically
    map_file_path = PathJoinSubstitution([
        TextSubstitution(text=os.path.join(
            get_package_share_directory('map_server'),
            'config'
        ) + os.sep),
        map_file_name
    ])

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_file_path}]
        )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    delay_duration = 5.0  # 5 seconds delay
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

    return LaunchDescription(
        [
            map_server_node,
            lifecycle_manager_node,
            delay_action
        ]
    )

# Additional handling for absolute path in the node
def get_map_file_path():
    package_name = 'other_package_name'  # Replace with your package name
    map_file = LaunchConfiguration('map_file')
    return os.path.join(get_package_share_directory(package_name), 'config', map_file.perform({}))