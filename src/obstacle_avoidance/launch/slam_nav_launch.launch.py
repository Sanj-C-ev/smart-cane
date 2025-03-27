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
        
        
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'base_frame': 'base_link'},
                {'scan_topic': '/scan'},
                {'mode': 'mapping'},
                {'odom': 'odom_rf2o'}
            ]
        ),
        
        # Start Navigation2 using IncludeLaunchDescription
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nav2_bringup"), "/launch", "/navigation_launch.py"
            ]),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
        
    ])
