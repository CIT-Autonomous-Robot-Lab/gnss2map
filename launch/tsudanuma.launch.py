# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, DeclareLaunchArgument
from launch.events import matches_action, Shutdown

from launch_ros.actions import LifecycleNode, Node
from launch_ros.events import lifecycle
from launch_ros.event_handlers import OnStateTransition
from launch.substitutions import TextSubstitution, LaunchConfiguration

from lifecycle_msgs.msg import Transition

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    declare_params_file = DeclareLaunchArgument(
        'params_file', 
        default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('gnss2map'), 
                'config', 'params', '')), 
            TextSubstitution(text='tsudanuma.param.yaml')
        ], 
        description='gnss2map param file path'
    )
    
    node = Node(
        package="gnss2map", 
        name="gauss_kruger_node", 
        executable="gauss_kruger_node", 
        parameters=[params_file], 
    )
    
    map_dir = os.path.join(get_package_share_directory('gnss2map'), 'config', 'map')
    map_file = os.path.join(
        map_dir, 'tsudanuma', 'map_tsudanuma.yaml')
    map_server_node = LifecycleNode(
        namespace='',
        name='map_server',
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
                {'yaml_filename': map_file}
        ]
    )

    emit_configuring_event = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    emit_activating_event = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    emit_shutdown_event = EmitEvent(
        event=Shutdown()
    )

    register_activating_transition = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            goal_state='inactive',
            entities=[
                emit_activating_event
            ],
        )
    )

    register_shutting_down_transition = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            goal_state='finalized',
            entities=[
                emit_shutdown_event
            ],
        )
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_params_file)
    ld.add_action(node)
    # ld.add_action(map_server_node)
    # ld.add_action(register_activating_transition)
    # ld.add_action(register_shutting_down_transition)
    # ld.add_action(emit_configuring_event)
    
    return ld
