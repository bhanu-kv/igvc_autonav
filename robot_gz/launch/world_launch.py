#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('robot_gz')

    description_pkg_name = 'robot_desc'
    install_dir = get_package_prefix(description_pkg_name)

    gz_models_path = os.path.join(pkg_robot, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH']+':'+install_dir+'/share' + ':' + gz_models_path

    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir+'/share' + ':' + gz_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH']+':'+install_dir + '/lib'

    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )

    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_robot, 'worlds', 'self_drive.world'), ''],
            description='sdf world files'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='simulator time'),
        gazebo
    ])


