from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    rf2o_pkg = get_package_share_directory('rf2o_laser_odometry')
    imu_data_pkg = get_package_share_directory('imu_data')
    mapping_nav_pkg = get_package_share_directory('mapping_navigation')

    # RPLIDAR launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_pkg, 'launch', 'sllidar_a1_launch.py')
        )
    )

    # RF2O Laser Odometry launch file
    rf2o_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(rf2o_pkg, 'launch', 'rf2o_laser_odometry.launch.py')
        ),
        launch_arguments={
            'log_level': 'error',  # Suppress INFO & WARN, keep ERROR
            'output': 'none'       # Optional: Also disable non-log output
        }.items()
    )

    # IMU Sensor Data Node 
    sensor_data_node = Node(
        package='imu_data',
        executable='sensor_data',
        name='sensor_data',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            # Add other IMU parameters here if needed
        }]
    )

    # SLAM Node 
    mapping = Node(
        package='mapping_navigation',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )

    path_planning = Node(
        package='mapping_navigation',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )

    local_planning = Node(
        package='mapping_navigation',
        executable='local_planner',
        name='local_planner',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )

    feedback_driver = Node(
        package='mapping_navigation',
        executable='feedback_driver',
        name='feedback_driver',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )
    controller = Node(
        package='mapping_navigation',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )
    goal_publisher = Node(
        package='mapping_navigation',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )

    return LaunchDescription([
        rplidar_launch,
        rf2o_launch,
        sensor_data_node,
        mapping,
        path_planning,
        local_planning,
        feedback_driver,
        controller,
        goal_publisher
    ])