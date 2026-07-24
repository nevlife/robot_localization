# SCV full localization: wheel + IMU + gated GPS, no magnetometer, no KISS-ICP.
#
# Heading backbone is gyro-integrated yaw; GPS (position + course heading) is fused ONLY
# during good-quality sections (gps_quality_gate). Dual filter + navsat_transform:
#
#   /current_speed --scaled_nhc_twist--> /wheel/twist_nhc
#   /ublox_gps_node/{fix,fix_velocity} --gps_quality_gate--> /gps/fix_gated, /gps/heading
#
#   ukf_odom  (scv_1.yaml)      : wheel + IMU            -> odom->base_link,  /odometry/filtered
#   ukf_map   (scv_global.yaml) : wheel + IMU + gated GPS-> map->odom,        /odometry/filtered_map
#   navsat    (scv_navsat.yaml) : gated fix + heading + ukf_map odom -> /odometry/gps
#
# Bag replay (drop the recorded /tf so it does not fight the filters' TF):
#   ros2 bag play ~/Downloads/SCV_20260701 --clock --remap /tf:=/tf_bag
#
# Launch args:
#   use_sim_time (default true)   set false on live hardware
#   wheel_scale  (default 0.925)  per-robot forward-speed calibration
#   max_pos_std  (default 1.0)    [m] GPS horizontal-std gate (RTK: ~0.1)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    wheel_scale = LaunchConfiguration('wheel_scale')
    max_pos_std = LaunchConfiguration('max_pos_std')

    share = get_package_share_directory('robot_localization')
    scv_1 = os.path.join(share, 'params', 'scv_1.yaml')
    scv_global = os.path.join(share, 'params', 'scv_global.yaml')
    scv_navsat = os.path.join(share, 'params', 'scv_navsat.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('wheel_scale', default_value='0.925'),
        DeclareLaunchArgument('max_pos_std', default_value='1.0'),

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

        # GPS quality gate: emit fix + course heading only while GPS is good.
        Node(
            package='robot_localization',
            executable='gps_quality_gate.py',
            name='gps_quality_gate',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'fix_topic': '/ublox_gps_node/fix',
                'fix_velocity_topic': '/ublox_gps_node/fix_velocity',
                'out_fix_topic': '/gps/fix_gated',
                'out_heading_topic': '/gps/heading',
                'frame_id': 'base_link',
                'min_status': 0,
                'max_pos_std': max_pos_std,
                'require_cov': True,
                'min_speed': 0.5,
                'max_speed_std': 0.5,
                'yaw_var': 0.02,
                'use_reverse_flip': False,
            }],
        ),

        # Local UKF (odom frame). Node name MUST match the top key in scv_1.yaml.
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node_odom',
            output='screen',
            parameters=[scv_1, {'use_sim_time': use_sim_time}],
        ),

        # Global UKF (map frame). Node name MUST match the top key in scv_global.yaml.
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node_map',
            output='screen',
            parameters=[scv_global, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered_map')],
        ),

        # navsat_transform: gated fix + gated heading + global odom -> /odometry/gps.
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[scv_navsat, {'use_sim_time': use_sim_time}],
            remappings=[
                ('gps/fix', '/gps/fix_gated'),
                ('imu', '/gps/heading'),
                ('odometry/filtered', '/odometry/filtered_map'),
            ],
        ),
    ])
