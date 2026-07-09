"""SCV localization bring-up: FAST-LIO + dual-EKF + navsat (replaces
tiny_localization).

  FAST-LIO (/velodyne_points + /vectornav/imu) -> /odometry/fast_lio
  wheel_odom_adapter (/hunter/velocity)        -> odometry/wheel
  ekf_odom  (world=odom) -> /odom (remapped)  + odom->base_link TF
  ekf_map   (world=map)  -> odometry/global   + map->odom TF
  navsat_transform (datum = graph-map node[0]) -> odometry/gps

NOTE the map->odom TF is owned HERE now — the global planner's
publishMapToOdomTransform must stay disabled (publish_map_odom_tf:=false).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rl_dir = get_package_share_directory('robot_localization')
    default_params = os.path.join(rl_dir, 'params', 'scv_dual_ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    with_fastlio = LaunchConfiguration('with_fastlio')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'with_fastlio', default_value='true',
            description='Start FAST-LIO here (false when it runs elsewhere '
                        'or when replaying a bag that already contains '
                        '/odometry/fast_lio)'),

        # LiDAR-inertial odometry (primary precision source)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('fast_lio'),
                             'launch', 'mapping.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'config_file': 'velodyne.yaml',
            }.items(),
            condition=IfCondition(with_fastlio),
        ),

        # GNSS antenna static TF — the ublox NavSatFix frame_id is
        # 'gnss_antenna'; navsat_transform refuses to compute GPS odometry
        # without base_link->antenna. Offsets are the antenna mount position
        # (measure on the vehicle; x/y matter, z is zeroed anyway).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gnss_antenna_tf',
            arguments=['0', '0', '0.9', '0', '0', '0',
                       'base_link', 'gnss_antenna'],
        ),

        # Hunter wheel speed -> Odometry twist
        Node(
            package='robot_localization',
            executable='wheel_odom_adapter.py',
            name='wheel_odom_adapter',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Local EKF: odom frame, feeds every /odom consumer downstream
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odom')],
        ),

        # Global EKF: map frame (GPS-anchored), owns map->odom TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),

        # GPS -> map-frame odometry (datum = graph-map node[0])
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('gps/fix', '/ublox_gps_node/fix'),
                ('imu', '/vectornav/imu'),
                ('odometry/filtered', 'odometry/global'),
            ],
        ),
    ])
