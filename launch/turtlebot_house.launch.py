# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution


def generate_launch_description():    
    turltlebot_house_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            TextSubstitution(text=os.path.join(
                    get_package_share_directory('gnss2map'), 
                    'launch', 'tsudanuma.launch.py'))]),
        launch_arguments={
            "params_file": [
            TextSubstitution(text=os.path.join(
                    get_package_share_directory('gnss2map'), 
                    'config', 'params', 'turtlebot_house.param.yaml'))],
        }.items()
    )
    ld = LaunchDescription()
    ld.add_action(turltlebot_house_launch)
    
    return ld
