# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    package = "gnss2map"
    config = os.path.join(
        get_package_share_directory(package), 
        "config", 
        "params", 
        "tsudanuma.param.yaml"
    )
    
    print(config)
    
    node = Node(
        package=package, 
        name="gauss_kruger_node", 
        executable="gauss_kruger_node", 
        parameters=[config], 
        # remappings=[("gnss/fix", "/fix")]
    )
    
    ld = LaunchDescription()
    ld.add_action(node)
    
    return ld
