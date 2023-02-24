#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim, Ryan Shim

import os
import random
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    robots_NUM = 4
    ROBOT_FOV = 120.0
    ROBOT_RANGE = 4.0
    launch_list = []
    SAVE_POS = False

    vision = Node(
            package='pf_coverage',
            node_executable='turtlebot_supervisor',
            parameters=[{"ROBOTS_NUM": robots_NUM}, 
                        {"ROBOT_FOV": ROBOT_FOV},
                        {"ROBOT_RANGE": ROBOT_RANGE},
                        {"SAVE_POS": SAVE_POS}],
            remappings=[("/vrpn_client_node/turtle3/pose", "/vrpn_client_node/turtle4/pose"),
                        ("/supervisor/robot4/pose", "/supervisor/robot3/pose")],
            output='screen')
    
    launch_list.append(vision)

    return LaunchDescription(launch_list)