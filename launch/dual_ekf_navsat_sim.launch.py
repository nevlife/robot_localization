# Dual-EKF + navsat for the Webots scout sim.
#
# Brings up:
#   ekf_filter_node_odom  KISS-ICP + IMU        -> odom -> base_footprint,  publishes odometry/local
#   ekf_filter_node_map   KISS-ICP + IMU + GPS  -> map  -> odom,            publishes odometry/global
#   navsat_transform      /gps/fix + /imu + odometry/global -> odometry/gps
#
# KISS-ICP is launched separately:
#   ros2 launch kiss_icp odometry.launch.py lidar_odom_frame:=odom base_frame:=base_footprint
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("robot_localization"),
        "params",
        "dual_ekf_navsat_sim.yaml",
    )
    use_sim_time = {"use_sim_time": True}

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[params, use_sim_time],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[params, use_sim_time],
            remappings=[("odometry/filtered", "odometry/global")],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[params, use_sim_time],
            remappings=[
                ("imu", "imu"),
                ("gps/fix", "gps/fix"),
                ("odometry/filtered", "odometry/global"),
            ],
        ),
    ])
