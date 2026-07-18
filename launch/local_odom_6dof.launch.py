#!/usr/bin/env python3
"""local_odom_6dof: 6DOF local odometry from wheel speed + IMU gyro rates.

Brings up:
  - current_speed_to_twist : /current_speed (Float64) -> /wheel/twist (TwistWithCovarianceStamped)
  - ukf_local_odom         : robot_localization UKF, publishes odom -> base_link (6DOF)

This replaces the KISS-ICP + 2D UKF stack. FastDEM consumes odom -> base_link and now
sees the real pitch/roll, which fixes the bumpy elevation seen near slope transitions.

Do NOT launch KISS-ICP or any other odom -> base_link TF source alongside this:
this UKF must be the single owner of that transform.

See params/local_odom_6dof.yaml for the fusion config.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = os.path.join(
        get_package_share_directory('robot_localization'),
        'params',
        'local_odom_6dof.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_localization',
            executable='current_speed_to_twist.py',
            name='current_speed_to_twist',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_local_odom',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),
    ])
