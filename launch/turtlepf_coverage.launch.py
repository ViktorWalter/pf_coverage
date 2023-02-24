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
    # AREA_SIZE_x = 3.5
    # AREA_SIZE_y = 2.8
    # AREA_LEFT = -1.2
    # AREA_BOTTOM = -1.2
    AREA_SIZE_x = 5.0
    AREA_SIZE_y = 4.0
    AREA_LEFT = -2.0
    AREA_BOTTOM = -1.8
    ROBOT_RANGE = 4.0
    ROBOT_FOV = 120.0

    # MODE: 0 = coverage, 1 = milling
    MODE = 0
    SAVE_POS = True

    ld = []
    # supervisor
    # n = Node( 
    #         package = "turtlebot3_coverage",
    #         node_executable = "coverage_distributed_neighmaster",
    #         parameters=[{"ROBOTS_NUM": ROBOTS_NUM}, {"AREA_SIZE_x": AREA_SIZE_x}, {"AREA_SIZE_y": AREA_SIZE_y}, {"AREA_LEFT": AREA_LEFT}, {"AREA_BOTTOM": AREA_BOTTOM}],
    #         output='screen')

    # ld.append(n)

    xg = [4.0, 4.0, -4.0, -4.0, 0.0, 0.0, 6.0, -6.0]
    yg = [-4.0, 4.0, 4.0, -4.0, -6.0, 6.0, 0.0, 0.0]

    for i in range(0,ROBOTS_NUM):
        n = Node(
            package='pf_coverage',
            node_executable='turtlepf_coverage',
            parameters=[{"ROBOTS_NUM": ROBOTS_NUM}, 
                        {"ROBOT_RANGE": ROBOT_RANGE}, 
                        {"ROBOT_FOV": ROBOT_FOV}, 
                        {"AREA_SIZE_x": AREA_SIZE_x}, 
                        {"AREA_SIZE_y": AREA_SIZE_y}, 
                        {"AREA_LEFT": AREA_LEFT}, 
                        {"AREA_BOTTOM": AREA_BOTTOM}, 
                        {"ROBOT_ID": i}, 
                        {"MODE": MODE}, 
                        {"GOAL_X": xg[i]}, 
                        {"GOAL_Y":yg[i]},
                        {"SAVE_POS": SAVE_POS}],
            remappings=[("/vrpn_client_node/turtle3/pose", "/vrpn_client_node/turtle4/pose"),
                        ("/turtle3/cmd_vel", "/turtle4/cmd_vel")],
            output='screen')

        ld.append(n)

    
    return LaunchDescription(ld)