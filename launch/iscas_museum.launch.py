# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction

from launch_ros.actions import  Node
from launch.substitutions import TextSubstitution, LaunchConfiguration

import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    declare_params_file = DeclareLaunchArgument(
        'params_file', 
        default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('gnss2map'), 
                'config', 'params', 'iscas_museum.param.yaml')), 
        ], 
        description='Params file path'
    )
    
    load_nodes = GroupAction(
        actions=[
            Node(
                package="gnss2map", 
                name="gauss_kruger_node", 
                executable="gauss_kruger_node", 
                parameters=[params_file]), 
            Node(
                package="gnss2map", 
                name="gnss_poser_node", 
                executable="gnss_poser_node", 
                parameters=[params_file])
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_params_file)
    ld.add_action(load_nodes)
    
    return ld
