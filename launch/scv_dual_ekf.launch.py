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
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
            'publish_antenna_tf', default_value='false',
            description='base_link->gnss_antenna static TF. The hunter URDF '
                        'ALREADY publishes it (measured: +0.020, 0.000, '
                        '+0.795) — leave false on the vehicle to avoid a '
                        'duplicate publisher; set true only for bag replay '
                        'without /tf_static.'),
        DeclareLaunchArgument(
            'with_fastlio', default_value='true',
            description='Start the LiDAR-inertial odometry here (false when '
                        'it runs elsewhere or when replaying a bag that '
                        'already contains /odometry/fast_lio). Which '
                        'algorithm starts is chosen by lio_source.'),
        DeclareLaunchArgument(
            'lio_source', default_value='fastlio',
            description='LiDAR-inertial odometry algorithm: '
                        'fastlio (FAST-LIO2, default/field-proven) | '
                        'fasterlio (Faster-LIO, iVox) | '
                        'rko (RKO-LIO). All three publish the SAME topic '
                        '/odometry/fast_lio in the odom frame so ekf_odom '
                        'fuses them identically — do not remap downstream. '
                        'None of them may publish odom->base_link TF (the '
                        'EKF owns it).'),
        DeclareLaunchArgument(
            'map_anchor_pcd', default_value='0',
            description='1/true: hybrid anchoring — consume /pcd/global_pose '
                        '(main-line PCD map matcher, run unmodified alongside) '
                        'as the anchor source while GPS is degraded (no RTK). '
                        'Requires the SCV_NEW_MAP0721-family map whose georef '
                        'offset matches map_anchor pcd_offset_e/n.'),
        DeclareLaunchArgument(
            'scan_yaw_init', default_value='0',
            description='1/true: 정지 스캔 정합 초기화 소비 — '
                        'standstill_yaw_init.py 가 발행하는 '
                        '/scan_yaw_init/pose 를 앵커+yaw 시드로 사용 '
                        '(pcd_offset 0, pcd_yaw_init on). '
                        "'10 m 전진 수렴' 절차의 정지 대체 (2026-08-13)."),
        DeclareLaunchArgument(
            'anchor_rtk_gate_m', default_value='3.0',
            description='RTK re-trust innovation gate [m]: an RTK-flagged '
                        'fix regains snap authority only after rtk_gate_n '
                        'consecutive samples within this distance of the '
                        'current estimate (false "recovered" fixes warped '
                        'the graph map, 2026-07-30 field). <=0 disables.'),
        DeclareLaunchArgument(
            'anchor_yaw_offset', default_value='1.8012',
            description='Static yaw added to the IMU absolute yaw [rad]. '
                        '1.8012 rad = +103.2 deg — the mounting rotation '
                        'that remains after the 2026-08-05 magnetic anchor '
                        'restoration (VPE ABSOLUTE + HSI applied). It is a '
                        'SEED, not a calibration: post-analysis of the three '
                        '08-05 sessions measured +104.4 / +117.7 / +94.1 deg '
                        '(span 23.6 deg, and 11-21 deg drift at the SAME 5 m '
                        'grid cell within 30 min), so the true value wanders '
                        'with local magnetic distortion. Seeding it still '
                        'beats 0 (100 deg error at boot vs ~10-20), but '
                        'anchor_yaw_autocal MUST stay on to trim the rest '
                        'and the RC forward gate before autonomy stays '
                        'MANDATORY. Repeatability across a power cycle is '
                        'still UNVERIFIED (the three sessions shared one '
                        'sensor power-on). Set 0 for chamber fault '
                        'injection / A-B.'),
        DeclareLaunchArgument(
            'anchor_yaw_autocal', default_value='true',
            description='Online yaw-offset estimation from GPS course vs '
                        'IMU yaw during confirmed forward motion (see '
                        'YawAutocal in map_anchor_node.py). false = rollback '
                        'to raw IMU yaw.'),
        DeclareLaunchArgument(
            'max_slew_mps', default_value='0.5',
            description='Anchor motion ceiling [m/s]. The map->odom offset '
                        'may not outrun the platform: on 2026-07-30 a '
                        'degraded fix stream (cov 100-420 m2) slid it at '
                        '5 m/s vs the vehicle 1.3 m/s. 0 disables (old '
                        'per-sample-only behaviour).'),
        DeclareLaunchArgument(
            'cov_ref_m2', default_value='1.0',
            description='Reported variance at which the anchor EMA runs at '
                        'its nominal time constant; larger reported variance '
                        'slows anchoring proportionally (inverse-variance '
                        'weighting). Huge value disables the weighting.'),
        DeclareLaunchArgument(
            'gate_max_radius_m', default_value='5000.0',
            description='GNSS sanity gate radius around the map datum; fixes '
                        'farther than this are dropped before navsat (blocks '
                        'receiver cold-start garbage). Raise to effectively '
                        'disable for A/B testing.'),

        # LiDAR-inertial odometry (primary precision source).
        # Three interchangeable implementations, selected by lio_source.
        # Contract each must satisfy: publish nav_msgs/Odometry on
        # /odometry/fast_lio in the odom frame, and publish NO
        # odom->base_link TF (ekf_odom owns that transform).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('fast_lio'),
                             'launch', 'mapping.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'config_file': 'velodyne.yaml',
            }.items(),
            condition=IfCondition(PythonExpression([
                "'", with_fastlio, "'.lower() in ('true','1') and '",
                LaunchConfiguration('lio_source'), "' == 'fastlio'"])),
        ),

        # Faster-LIO: same iEKF as FAST-LIO2 but iVox instead of ikd-Tree.
        # Its node publishes on 'Odometry' (node-relative), so remap.
        Node(
            package='faster_lio',
            executable='run_mapping_online',
            name='laserMapping',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('faster_lio'),
                    'config', 'velodyne_scv.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('Odometry', '/odometry/fast_lio')],
            condition=IfCondition(PythonExpression([
                "'", with_fastlio, "'.lower() in ('true','1') and '",
                LaunchConfiguration('lio_source'), "' == 'fasterlio'"])),
        ),

        # RKO-LIO: IMU-loose, no per-sensor tuning. publish_odom_tf:=false
        # keeps the EKF's ownership of odom->base_link (the package would
        # otherwise broadcast it and fight the filter).
        Node(
            package='rko_lio',
            executable='online_node',
            name='rko_lio',
            output='screen',
            parameters=[
                # scv.yaml 이 아니다: 그 파일은 rko 자체 런치용 평문 형식이라
                # Node(parameters=[...]) 로 넘기면 rcl 파싱이 실패한다.
                PathJoinSubstitution([FindPackageShare('rko_lio'),
                             'config', 'scv_ros_params.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            condition=IfCondition(PythonExpression([
                "'", with_fastlio, "'.lower() in ('true','1') and '",
                LaunchConfiguration('lio_source'), "' == 'rko'"])),
        ),

        # GNSS antenna static TF fallback. navsat_transform refuses to
        # compute GPS odometry without base_link->gnss_antenna (NavSatFix
        # frame_id). On the real vehicle the hunter URDF publishes it
        # (base_link -> sensors_base_link -> gnss_antenna = +0.020, 0.000,
        # +0.795 — verified from the bag /tf_static), so this fallback stays
        # OFF by default and exists for URDF-less bench/replay setups only.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gnss_antenna_tf',
            arguments=['0.020', '0', '0.795', '0', '0', '0',
                       'base_link', 'gnss_antenna'],
            condition=IfCondition(LaunchConfiguration('publish_antenna_tf')),
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

        # Map anchoring: owns map->odom TF + /odometry/global.
        # NOT an EKF: the map EKF diverged km-scale on the 2026-07-18 field
        # run (out-of-order navsat stamps vs 100 Hz IMU excited its
        # unobserved velocity/acceleration states — see map_anchor_node.py).
        # A complementary filter estimates exactly what anchoring needs (a
        # slowly-varying rigid map->odom transform) and nothing else.
        Node(
            package='robot_localization',
            executable='map_anchor_node.py',
            name='map_anchor',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[{
                'use_sim_time': use_sim_time,
                'pcd_pose_topic': PythonExpression(
                    ["'/scan_yaw_init/pose' if '",
                     LaunchConfiguration('scan_yaw_init'),
                     "' in ('1', 'true', 'True') else "
                     "('/pcd/global_pose' if '",
                     LaunchConfiguration('map_anchor_pcd'),
                     "' in ('1', 'true', 'True') else '')"]),
                # 정지 정합 포즈는 우리 map 프레임 그대로 — 오프셋 0
                'pcd_offset_e': PythonExpression(
                    ["0.0 if '", LaunchConfiguration('scan_yaw_init'),
                     "' in ('1', 'true', 'True') else 45.518"]),
                'pcd_offset_n': PythonExpression(
                    ["0.0 if '", LaunchConfiguration('scan_yaw_init'),
                     "' in ('1', 'true', 'True') else 26.015"]),
                'pcd_yaw_init': PythonExpression(
                    ["'", LaunchConfiguration('scan_yaw_init'),
                     "' in ('1', 'true', 'True')"]),
                'rtk_gate_m': ParameterValue(
                    LaunchConfiguration('anchor_rtk_gate_m'),
                    value_type=float),
                'yaw_offset': ParameterValue(
                    LaunchConfiguration('anchor_yaw_offset'),
                    value_type=float),
                'yaw_autocal': ParameterValue(
                    LaunchConfiguration('anchor_yaw_autocal'),
                    value_type=bool),
                'max_slew_mps': ParameterValue(
                    LaunchConfiguration('max_slew_mps'), value_type=float),
                'cov_ref_m2': ParameterValue(
                    LaunchConfiguration('cov_ref_m2'), value_type=float),
            }],
        ),

        # GNSS sanity gate: drops cold-start garbage (e.g. a fix 110 km out
        # right after receiver power-up) BEFORE it can kick the map EKF,
        # which deliberately has no rejection threshold. Datum must match
        # navsat's datum in scv_dual_ekf.yaml.
        Node(
            package='robot_localization',
            executable='gps_fix_gate.py',
            name='gps_fix_gate',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/ublox_gps_node/fix',
                'output_topic': '/gps/fix_gated',
                'datum_lat': 35.91361,
                'datum_lon': 128.80308,
                'max_radius_m': LaunchConfiguration('gate_max_radius_m'),
            }],
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
                ('gps/fix', '/gps/fix_gated'),
                ('imu', '/vectornav/imu'),
                ('odometry/filtered', 'odometry/global'),
            ],
        ),
    ])
