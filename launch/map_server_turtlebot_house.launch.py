# SPDX-FileCopyrightText: 2023 MakotoYoshigoe
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

# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution


def generate_launch_description():    
    tsukuba_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            TextSubstitution(text=os.path.join(
                    get_package_share_directory('gnss2map'), 
                    'launch', 'map_server_tsudanuma.launch.py'))]),
        launch_arguments={
            "map_path": [
            TextSubstitution(text=os.path.join(
                    get_package_share_directory('gnss2map'), 
                    'config', 'map', 'turtlebot_house', 'map_turtlebot_house.yaml'))],
        }.items()
    )
    ld = LaunchDescription()
    ld.add_action(tsukuba_launch)
    
    return ld
