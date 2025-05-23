from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'simple_navigation'

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
            'controller = simple_navigation.controller:main',
            'driver = simple_navigation.feedback_driver:main',
            'haptic_driver = simple_navigation.haptic_driver:main',
            'kinesthetic_driver = simple_navigation.kinesthetic_driver:main',
            'ultrasonic_obstacle = simple_navigation.ultrasonic_haptic:main',
            'lidar_us_obstacles = simple_navigation.lidar_ultrasonic_haptic:main',
            'ultrasonic_2_sensors = simple_navigation.ultrasonic_haptic_kinesthetic:main ', 
            'final_navigation = simple_navigation.combined_haptic_kinesthetic:main'           

        ],
    },
)
