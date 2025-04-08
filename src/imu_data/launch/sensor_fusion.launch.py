from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('imu_data'),
                    'config',
                    'ekf_gps_imu_odom.yaml'
                ])
            ],
            remappings=[
                ('odom0', '/odom_rf2o'),
                ('imu0', '/imu/data'),
                ('navsat0', '/gps')
            ]
        ),
        
        # Static Transform Publisher (new style arguments)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['--x', '0.1', '--y', '0', '--z', '0.1',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'odom', '--child-frame-id', 'base_link'],
            output='screen'
        )
    ])
