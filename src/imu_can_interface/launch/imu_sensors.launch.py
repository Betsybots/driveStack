# ============================================================
#  IMU — ROS 2 Launch File
# ============================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── package path ──
    pkg_dir = get_package_share_directory('imu_can_interface')

    # ── launch arguments ──
    can_iface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can1',
        description='CAN interface name'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'imu_params.yaml'),
        description='Path to YAML config file'
    )

    # ── node ──
    imu_node = Node(
        package='imu_can_interface',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'can.interface': LaunchConfiguration('can_interface')},
        ],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data',     '/imu/data'),
        ],
    )


    return LaunchDescription([
        can_iface_arg,
        config_file_arg,
        imu_node,
    ])
