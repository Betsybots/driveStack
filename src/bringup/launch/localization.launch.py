import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _as_bool(value, name):
    normalized = value.strip().lower()
    if normalized in ('true', '1', 'yes', 'on'):
        return True
    if normalized in ('false', '0', 'no', 'off'):
        return False
    raise RuntimeError("Launch argument '{}' must be true or false".format(name))


def _launch_localization(context):
    use_ekf = _as_bool(context.launch_configurations['use_ekf'], 'use_ekf')
    with_lidar = (
        True if not use_ekf
        else _as_bool(context.launch_configurations['with_lidar'], 'with_lidar')
    )
    wheel_odom_topic = context.launch_configurations['wheel_odom_topic']
    imu_topic = context.launch_configurations['imu_topic']
    lidar_odom_topic = context.launch_configurations['lidar_odom_topic']
    output_topic = context.launch_configurations['output_topic']

    if not use_ekf:
        return [
            Node(
                package='bringup',
                executable='odometry_relay.py',
                name='lidar_odometry_relay',
                output='screen',
                parameters=[{
                    'input_topic': lidar_odom_topic,
                    'output_topic': output_topic,
                }],
            )
        ]

    parameters = [
        context.launch_configurations['ekf_config_file'],
        {
            'odom0': wheel_odom_topic,
            'imu0': imu_topic,
        },
    ]

    if with_lidar:
        parameters.append({
            'odom1': lidar_odom_topic,
            'odom1_config': [
                True, True, False,
                False, False, True,
                False, False, False,
                False, False, False,
                False, False, False,
            ],
            'odom1_differential': False,
            'odom1_relative': False,
            'odom1_queue_size': 5,
            'odom1_pose_rejection_threshold': 5.0,
        })

    return [
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=parameters,
            remappings=[('odometry/filtered', output_topic)],
        )
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Fuse localization sensors with robot_localization EKF',
        ),
        DeclareLaunchArgument(
            'with_lidar',
            default_value='false',
            description='Include LiDAR odometry; automatically true when use_ekf is false',
        ),
        DeclareLaunchArgument(
            'ekf_config_file',
            default_value=os.path.join(bringup_share, 'config', 'localization_ekf.yaml'),
            description='Path to the wheel odometry and IMU EKF configuration',
        ),
        DeclareLaunchArgument(
            'wheel_odom_topic',
            default_value='/wheel_odom',
            description='Wheel odometry topic',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='IMU topic',
        ),
        DeclareLaunchArgument(
            'lidar_odom_topic',
            default_value='/Odometry',
            description='LiDAR odometry topic',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/FinalOdometry',
            description='Localization odometry output topic',
        ),
        OpaqueFunction(function=_launch_localization),
    ])
