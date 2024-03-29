# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.events import matches_action
from launch.events import Shutdown

from launch_ros.actions import LifecycleNode
from launch_ros.events import lifecycle
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition
from launch_ros.actions import Node

def generate_launch_description():
    gnss2map_dir = get_package_share_directory('gnss2map')
    map_dir = os.path.join(gnss2map_dir, 'config', 'map')
    map_file = os.path.join(
        map_dir, 'turtlebot_house', 'map_turtlebot_house.yaml')

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
    
    config = os.path.join(
        gnss2map_dir, 
        "config", 
        "params", 
        "turtlebot_house.param.yaml"
    )
    
    node = Node(
        package="gnss2map", 
        name="gauss_kruger_node", 
        executable="gauss_kruger_node", 
        parameters=[config], 
    )
    
    ld = LaunchDescription()
    ld.add_action(node)
    ld.add_action(map_server_node)
    ld.add_action(register_activating_transition)
    ld.add_action(register_shutting_down_transition)
    ld.add_action(emit_configuring_event)
    
    return ld
