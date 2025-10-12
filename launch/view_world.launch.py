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


def generate_launch_description():
    world_file_name = 'ray_tracing.world'
    package_dir = get_package_share_directory('ray_tracing_plugin')
    world = LaunchConfiguration('world')

    model_path = os.path.join(package_dir, 'models')

    gazebo_server_cmd_line = [
        'gz', 'sim', '-r', world]

    gazebo = ExecuteProcess(
        cmd=gazebo_server_cmd_line, output='screen')

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
    ])


if __name__ == '__main__':
    generate_launch_description()