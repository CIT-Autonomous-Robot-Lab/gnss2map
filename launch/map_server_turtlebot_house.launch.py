import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value='/home/ubuntu/ros2_ws/src/gnss2map/config/map/turtlebot_house/map_turtlebot_house.yaml',
        description='Full path to map yaml file to load')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    lifecycle_nodes = ['map_server']
    
    package = "gnss2map"
    config = os.path.join(
        get_package_share_directory(package), 
        "config", 
        "rviz", 
        "rviz.rviz"
    )

    launch_node = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d', config]), 
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{'yaml_filename': map_yaml_file}],
                output='screen'),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_use_sim_time)
    ld.add_action(launch_node)

    return ld
