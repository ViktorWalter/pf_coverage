#!/usr/bin/env python3
#


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix



def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROBOTS_NUM = 4
    AREA_SIZE_x = 40.0
    AREA_SIZE_y = 40.0
    AREA_LEFT = -20.0
    AREA_BOTTOM = -20.0

    ld = []
    # supervisor
    n = Node( 
            package = "pf_coverage",
            node_executable = "coverage_distributed_neighmaster",
            parameters=[{"ROBOTS_NUM": ROBOTS_NUM}, {"AREA_SIZE_x": AREA_SIZE_x}, {"AREA_SIZE_y": AREA_SIZE_y}, {"AREA_LEFT": AREA_LEFT}, {"AREA_BOTTOM": AREA_BOTTOM}],
            output='screen')

    ld.append(n)

    for i in range(1,ROBOTS_NUM):
        n = Node(
            package='pf_coverage',
            node_executable='coverage_distributed_singlenode',
            parameters=[{"ROBOTS_NUM": ROBOTS_NUM}, {"AREA_SIZE_x": AREA_SIZE_x}, {"AREA_SIZE_y": AREA_SIZE_y}, {"AREA_LEFT": AREA_LEFT}, {"AREA_BOTTOM": AREA_BOTTOM}, {"Robot_ID": i}],
            output='screen')

        ld.append(n)

    
    return LaunchDescription(ld)