from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'mapping_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share',package_name,'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanj_19',
    maintainer_email='ssanjeev17537@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_fused = mapping_navigation.mapping_fused:main',
            'slam_node_2 = mapping_navigation.mapping:main',
            'slam_node = mapping_navigation.mapping_costmap:main',
            'slam_node_3 = mapping_navigation.mapping_costmap_non_updating:main', 
            'goal_publisher = mapping_navigation.goal_publish:main',
            'path_planner = mapping_navigation.navigation_frontier_dlite:main',
            'path_planner_2 = mapping_navigation.maze_solving_navigation:main',
            'local_planner=mapping_navigation.local_planner_pure_pursuit:main',
            'feedback_driver=mapping_navigation.feedback_driver:main',
            'controller = mapping_navigation.controller:main'
        ],
    },
)
