#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: quangtuong.phung@uts.edu.au

import os
import random
from ament_index_python.packages import get_package_share_directory, get_package_prefix, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Pre-define variables
    root_pkg = 'spawn-env-rclcpp'
    environment = 'neo_track1'

    # Set environment variables
    pkg_gazebo_models = get_package_share_directory(root_pkg)
    gazebo_models_path = os.path.join(pkg_gazebo_models, 'models')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + pkg_gazebo_models + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_gazebo_models + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + pkg_gazebo_models + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = pkg_gazebo_models + '/lib'

    # Launch Gazebo
    default_world_path = os.path.join(get_package_share_path(root_pkg), 'worlds', f'{environment}.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': default_world_path,
        }.items()
    )

    return LaunchDescription([
        gazebo,
        use_sim_time,
    ])
