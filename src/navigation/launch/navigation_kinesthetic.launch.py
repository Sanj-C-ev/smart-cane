from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sllidar = get_package_share_directory('sllidar_ros2')
    #rf2o_pkg = get_package_share_directory('rf2o_laser_odometry')
    imu_data_pkg = get_package_share_directory('imu_data')
    navigation = get_package_share_directory('navigation')

    # RPLIDAR launch
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar, 'launch', 'rplidar_a1_launch.py')
        )
    )

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
    obstacle_avoiadance = Node(
        package='navigation',
        executable='final_navigation',
        name='final_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )


    feedback_driver = Node(
        package='navigation',
        executable='kinesthetic_driver',
        name='kinesthetic_driver',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )
    controller = Node(
        package='navigation',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )
      

    return LaunchDescription([
        sllidar_launch,
        sensor_data_node,
        obstacle_avoiadance,
        feedback_driver,
        controller
    ])