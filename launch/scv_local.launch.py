# SCV local odometry (odom frame): wheel speed + IMU, no magnetometer, no KISS-ICP.
#
# Pipeline:
#   /current_speed (Float64) --scaled_nhc_twist--> /wheel/twist_nhc (Vx scaled + Vy=0 NHC)
#   /wheel/twist_nhc + /vectornav/imu --ukf(scv_1.yaml)--> /odometry/filtered + TF odom->base_link
#
# Bag replay (exclude the recorded /tf so it does not fight the UKF's odom->base_link):
#   ros2 bag play ~/Downloads/SCV_20260701 --clock \
#       --remap /tf:=/tf_bag
#   (keep /tf_static and /robot_description; --clock feeds sim time)
#
# Launch args:
#   use_sim_time (default true)  set false for live hardware
#   wheel_scale  (default 0.925) per-robot forward-speed calibration

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    wheel_scale = LaunchConfiguration('wheel_scale')

    scv_1 = os.path.join(
        get_package_share_directory('robot_localization'), 'params', 'scv_1.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('wheel_scale', default_value='0.925'),

        # Scalar wheel speed -> body-frame twist with nonholonomic constraint.
        Node(
            package='robot_localization',
            executable='scaled_nhc_twist.py',
            name='scaled_nhc_twist',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/current_speed',
                'output_topic': '/wheel/twist_nhc',
                'frame_id': 'base_link',
                'wheel_scale': wheel_scale,
                'vx_var': 0.05,
                'nhc_var': 0.001,
            }],
        ),

        # Local UKF. Node name MUST match the top key in scv_1.yaml.
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node_odom',
            output='screen',
            parameters=[scv_1, {'use_sim_time': use_sim_time}],
        ),
    ])
