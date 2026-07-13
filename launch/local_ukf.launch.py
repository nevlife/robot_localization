# Dedicated launch for the local UKF odometry (scv_1.yaml).
#
# Single UKF fusing KISS-ICP (kiss/odometry) + IMU (imu) into a smooth
# odom -> base_footprint pose (Ackermann, 2D, no magnetometer). See
# params/scv_1.yaml for the fusion config.
#
# This launch brings up the UKF node only; the sim and KISS-ICP run separately:
#   ros2 launch scout_bringup teleop_launch.py                            # sim: /imu, lidar, (wheel odom)
#   ros2 launch kiss_icp odometry.launch.py topic:=/velodyne/point_cloud  # kiss/odometry (publish_odom_tf false)
#   ros2 launch robot_localization local_ukf.launch.py                    # this UKF
#
# The UKF output (odometry/filtered) is remapped to odometry/local so it is a
# drop-in for the dual-EKF local output that terrain_analysis consumes. Do not run
# this together with dual_ekf_navsat_sim.launch.py: both own the odom -> base TF.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = os.path.join(
        get_package_share_directory("robot_localization"),
        "params",
        "scv_1.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="robot_localization",
            executable="ukf_node",
            name="ukf_filter_node_odom",
            output="screen",
            parameters=[params, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
    ])
