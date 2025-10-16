# /*******************************************************************************
# * Copyright 2025 Intelligents Robotics Lab - URJC.
# *
# *******************************************************************************/

# /* Author: Juan S. Cely */

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    world_file_name = 'ray_tracing.world'
    package_dir = get_package_share_directory('ray_tracing_plugin')
    world = LaunchConfiguration('world')

    model_path = os.path.join(package_dir, 'models')

    gazebo_server_cmd_line = [
        'gz', 'sim', '-r', world]

    gazebo = ExecuteProcess(
        cmd=gazebo_server_cmd_line, output='screen')
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[
            {
                'config_file': os.path.join(
                    package_dir, 'config', 'bridge.yaml'
                ),
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    # Rviz config and launching
    rviz_config_file = os.path.join(
                    package_dir, 'rviz', 'ray_tracing_plugin.rviz')

    # Rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(package_dir, 'worlds', world_file_name), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        gazebo,
        bridge,
        rviz_node,
    ])


if __name__ == '__main__':
    generate_launch_description()