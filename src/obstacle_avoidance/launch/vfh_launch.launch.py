import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='2.0', description='Goal X coordinate'),
        DeclareLaunchArgument('goal_y', default_value='3.0', description='Goal Y coordinate'),

        # Start RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),

        # Start RF2O Laser Odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("rf2o_laser_odometry"), "/launch", "/rf2o_laser_odometry.launch.py"
            ]),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),

        # Publish static transform: odom_rf2o -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom_rf2o', 'base_link']
        ),

        # Publish static transform: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # Start Obstacle Avoidance Node
        #Node(
         #   package='obstacle_avoidance',
          #  executable='kinesthetic_serial',
           # name='kinesthetic',
           # output='screen'
        #),
    ])
