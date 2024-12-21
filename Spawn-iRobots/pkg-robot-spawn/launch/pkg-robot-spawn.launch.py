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
    root_pkg = 'pkg-robot-spawn'
    robot_pkg = 'mp400-robot-builder'
    robot_name = 'mp_400'
    environment = 'neo_track1'

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Set environment variables
    install_dir = get_package_prefix(robot_pkg)
    pkg_gazebo_models = get_package_share_directory(root_pkg)
    gazebo_models_path = os.path.join(pkg_gazebo_models, 'models')

    # os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join([
    #     os.environ.get('GAZEBO_MODEL_PATH', ''),
    #     install_dir + '/share',
    #     gazebo_models_path
    # ])

    # os.environ['GAZEBO_PLUGIN_PATH'] = os.pathsep.join([
    #     os.environ.get('GAZEBO_PLUGIN_PATH', ''),
    #     install_dir + '/lib'
    # ])

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Spawn the robot
    position = [0.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]
    entity_name = f'{robot_name}-{str(int(random.random()*100000))}'

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', entity_name,
            '-file', os.path.join(get_package_share_directory(robot_pkg), 'urdf', f'{robot_name}.urdf'),
            '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
            '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
        ]
    )

    # Publish robot state
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher_node',
    #     emulate_tty=True,
    #     parameters=[
    #        {
    #             'use_sim_time': True,
    #             'robot_description': os.path.join(get_package_share_directory(robot_pkg), 'urdf', robot_name + '.urdf')
    #         }
    #     ],
    #     output="screen"
    # )

    # urdf = os.path.join(get_package_share_directory(''), 'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf')

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf = os.path.join(get_package_share_directory(robot_pkg), 'urdf', robot_name + '.urdf')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[{
             'use_sim_time': True,
             # 'robot_description': urdf, 
             }],
        arguments=[urdf])

    # Visualize in RViz
    rviz_config_dir = os.path.join(get_package_share_directory(robot_pkg), 'rviz', 'robotTF.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

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
        robot_state_publisher_node,
        spawn_robot,
        # rviz_node,
    ])

