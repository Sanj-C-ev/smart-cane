from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rplidar_pkg = get_package_share_directory('rplidar_ros')
    #rf2o_pkg = get_package_share_directory('rf2o_laser_odometry')
    imu_data_pkg = get_package_share_directory('imu_data')
    simple_navigation = get_package_share_directory('simple_navigation')

    # RPLIDAR launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg, 'launch', 'rplidar_a1_launch.py')
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
        package='simple_navigation',
        executable='final_navigation',
        name='final_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )


    feedback_driver = Node(
        package='simple_navigation',
        executable='haptic_driver',
        name='haptic_driver',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )
    controller = Node(
        package='simple_navigation',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
           
        }]
    )
    
    
    

    return LaunchDescription([
        rplidar_launch,
        sensor_data_node,
        obstacle_avoiadance,
        feedback_driver,
        controller
    ])