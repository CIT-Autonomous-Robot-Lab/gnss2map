# Copyright 2023 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    loc_map_yaml_file = LaunchConfiguration('loc_map')
    nav_map_yaml_file = LaunchConfiguration('nav_map')
    params_file = LaunchConfiguration('params_file')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    declare_use_composition = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')
    declare_arg_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set "true" to launch rviz.')
    
    map_dir = os.path.join(get_package_share_directory('gnss2map'), 'config', 'map', 'tsudanuma')
    declare_loc_map_yaml = DeclareLaunchArgument(
        'loc_map', default_value=os.path.join(map_dir, 'map_tsudanuma.yaml'),
        description='Full path to map yaml file for localization to load')
    
    declare_nav_map_yaml = DeclareLaunchArgument(
        'nav_map', default_value=os.path.join(map_dir, 'navigation', 'map_tsudanuma_campus.yaml'),
                description='Full path to map yaml file for navigation to load')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('gnss2map'),
                'config', 'params', 'gauss_kruger.param.yaml'),
                description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_container_name = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    container_name_full = (namespace, '/', container_name)

    param_substitutions = {
        'use_sim_time': use_sim_time,}
        # 'loc_map_yaml_filename': loc_map_yaml_file, 
        # 'nav_map_yaml_filename': nav_map_yaml_file}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    lifecycle_nodes = [
        'loc_map_server',
        ]

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package="gnss2map", 
                name="gauss_kruger_node", 
                executable="gauss_kruger_node", 
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('gnss/fix', '/vps/fix')]), 
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='loc_map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {"yaml_filename": loc_map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings+[('map', '/map/localization')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='loc_map_server',
                parameters=[configured_params],
                remappings=remappings+[('map', '/map/localization')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    rviz_config_file = os.path.join(get_package_share_directory('raspicat_navigation'), 
                                    'config', 'rviz', 'nav2.rviz')
    rviz2 = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        ros_arguments=['--log-level', 'WARN'],
        condition=IfCondition(use_rviz))

    ld = LaunchDescription()


    ld.add_action(declare_namespace)
    ld.add_action(declare_loc_map_yaml)
    ld.add_action(declare_use_composition)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_autostart)
    ld.add_action(declare_params_file)
    ld.add_action(declare_log_level)
    ld.add_action(declare_arg_use_rviz)

    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    
    ld.add_action(rviz2)
    
    return ld
  
