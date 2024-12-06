import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = 'robot_desc.urdf'
    pkg_name = 'robot_desc'
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    robot_desc_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', urdf_file)

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_node',
            emulate_tty=True,
            parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ',robot_desc_path])}],
            output='screen'
            )

    joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_node',
            parameters=[{'use_sim_time':True}],
            output='screen'
            )

    position = [4.340573, -18.490131, 0.0]
    orientation = [0.0, 0.0, 0.0]

    robot_name = 'bot'

    spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity',
                       robot_name,
                       '-x', str(position[0]), 
                       '-y', str(position[1]), 
                       '-z', str(position[2]),
                       '-R', str(orientation[0]),
                       '-P', str(orientation[1]),
                       '-Y', str(orientation[2]),
                       '-topic', '/robot_description'
                       ]
            )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'config.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
            )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py'),
        )
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py'),
        )
    )

    camera_transform = Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen" ,
                arguments=["0.48204", "0.0", "-1.32881", "0", "-0.3263", "0", "camera_link", "chassis"],
                )

    camera_short_transform = Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen" ,
                arguments=["0.1", "0.0", "-1.401425727958", "0", "-0.418879", "0", "camera_short_link", "chassis"],
                )

    return LaunchDescription(
            [
                DeclareLaunchArgument(
                    'use_sim_time',
                    default_value='True',
                    description='custom config for nav2_bringup'),

                DeclareLaunchArgument(
                    'slam_params_file',
                    default_value=[os.path.join(get_package_share_directory(pkg_name), 'rviz', 'mapper_params_online_async.yaml'),''],
                    description='custom config for SLAM'),

                robot_state_publisher_node,
                joint_state_publisher_node,
                spawn_robot,
                rviz,
                # slam_toolbox,
                DeclareLaunchArgument(
                    'params_file',
                    default_value=[os.path.join(get_package_share_directory(pkg_name), 'rviz', 'nav2_params.yaml'),''],
                    description='custom config for nav2_bringup'),
                # nav2_bringup,
            ]
            ) 

