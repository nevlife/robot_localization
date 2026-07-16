# Dual-UKF + navsat localization for the Hunter 2 (Ackermann) Gazebo sim.
#
# Brings up (all fed dual_ukf_navsat_sim.yaml):
#   ukf_filter_node_odom  wheel + IMU        -> odom -> base_link,  publishes odometry/local
#   ukf_filter_node_map   wheel + IMU + GPS  -> map  -> odom,        publishes odometry/global
#   navsat_transform      /gps/fix + /imu + odometry/global -> odometry/gps
#
# No KISS-ICP. Inputs are the live sim topics: /odometry/wheel, /imu, /gps/fix.
# This node owns odom->base_link and map->odom; the sim wheel odom is topic-only
# (no TF), so there is no conflict.
#
# Prereqs (see the yaml header): the /imu frame_id must resolve in TF, and the
# /odometry/wheel covariances should be nonzero, or the UKF drops/overtrusts inputs.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("robot_localization"),
        "params",
        "dual_ukf_navsat_sim.yaml",
    )
    use_sim_time = {"use_sim_time": True}

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ukf_node",
            name="ukf_filter_node_odom",
            output="screen",
            parameters=[params, use_sim_time],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
        Node(
            package="robot_localization",
            executable="ukf_node",
            name="ukf_filter_node_map",
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
                ("imu", "/imu"),
                ("gps/fix", "/gps/fix"),
                ("odometry/filtered", "odometry/global"),
            ],
        ),
    ])
