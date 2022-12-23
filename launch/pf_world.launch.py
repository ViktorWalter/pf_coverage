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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_file_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    robots_NUM = 4
    launch_list = []

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/gazebo.launch.py']),
            launch_arguments={'gui': 'true'}.items(),
        )

    launch_list.append(gazebo)


    # x_pos = [-2, -3, 2, 2]
    # y_pos = [-2, 4, 1, 2]
    # theta = [0.25 * 3.14, 0, 0, 0]

    x_pos = [-5.0, 5.0, 5.0, -5.0]
    y_pos = [-5.0, -5.0, 5.0, 5.0]
    theta = 0.25 * 3.14
        

    for i in range(robots_NUM):
        spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-entity', 'turtlebot'+str(i), '-database', 'turtlebot3_' + TURTLEBOT3_MODEL, '-robot_namespace', 'turtlebot'+str(i),
                                    '-x', str(float(x_pos[i])), '-y', str(float(y_pos[i])), '-z', str(0.1), '-Y', str((2*i+1)*theta),
                            ],
                            output='screen')

        launch_list.append(spawn_entity)

    return LaunchDescription(launch_list)