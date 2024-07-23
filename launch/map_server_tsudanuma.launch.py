# SPDX-FileCopyrightText: 2023 MakotoYoshigoe
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, EmitEvent, DeclareLaunchArgument, RegisterEventHandler
from launch.events import matches_action, Shutdown

from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.events import lifecycle
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition
from launch.substitutions import TextSubstitution, LaunchConfiguration

from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map_path')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
    
    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', 
        default_value = 'true'
    )
    
    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('gnss2map'), 'config', 'map', 'tsudanuma', '')), 
            TextSubstitution(text='map_tsudanuma.yaml')],
        description='Full path to map yaml file to load')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', 
        default_value='true'
    )
    
    declare_namespace = DeclareLaunchArgument(
        'namespace', 
        default_value=TextSubstitution(text='')
    )
    
    lifecycle_nodes = ['map_server']
    
    launch_node = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time), 
            Node(
                namespace=namespace,
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{'yaml_filename': map_path}],
                output='screen',
            ), 
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', 
                        TextSubstitution(text=os.path.join(get_package_share_directory('gnss2map'), 
                        'config', 'rviz', 'rviz.rviz'))], 
                condition=IfCondition(use_rviz)
            ),
            Node(
                namespace=namespace,
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_map_path)
    ld.add_action(declare_namespace)
    
    ld.add_action(launch_node)

    return ld
