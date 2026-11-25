from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('time_strategy', default_value='C', choices=['A', 'B', 'C']),
        DeclareLaunchArgument('planner_id', default_value='GridBased'),
        Node(package='frontier_exploration', executable='map_listener',
             name='map_listener', output='screen', parameters=[clock]),
        Node(package='frontier_exploration', executable='frontier_detector',
             name='frontier_detector', output='screen', parameters=[clock]),
        Node(package='frontier_exploration', executable='frontier_selector',
             name='frontier_selector', output='screen', parameters=[clock, {
                 'time_strategy': LaunchConfiguration('time_strategy'),
                 'planner_id': LaunchConfiguration('planner_id'),
             }]),
    ])
