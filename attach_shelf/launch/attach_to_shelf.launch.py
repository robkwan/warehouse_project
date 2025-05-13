import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_description = "attach_shelf"

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'default.rviz')

    final_approach = LaunchConfiguration('final_approach', default='true')

    obstacle = LaunchConfiguration('obstacle')

    degrees = LaunchConfiguration('degrees')

    declare_obstacle_arg = DeclareLaunchArgument(
            'obstacle',
            default_value='-0.3',
            description='Distance to consider an obstacle'
        )

    declare_degrees_arg = DeclareLaunchArgument(
            'degrees',
            default_value='90.0',
            description='Degrees to rotate'
        ) 

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription([
        declare_obstacle_arg,
        declare_degrees_arg,
        Node(
            package="attach_shelf",
            executable="pre_approach_v2",
            name="pre_approach_v2",
            parameters=[{"final_approach": final_approach}, 
                {"obstacle": obstacle},
                {"degrees": degrees}]
        ),
        Node(
            package="attach_shelf",
            executable="approach_service_server",
            name="approach_service_server"
        ),
        rviz_node
    ])

