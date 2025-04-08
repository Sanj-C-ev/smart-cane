from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'obstacle_avoidance'

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
    maintainer_email='sanj_19@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vfh_avoidance = obstacle_avoidance.vfh_with_haptic:main',
            'kinesthetic_cmd = obstacle_avoidance.kinesthetic_command:main',
            'kinesthetic_serial = obstacle_avoidance.kinesthetic_serial:main',
            'vfh_v2 = obstacle_avoidance.vfh_v2:main',
            'cmd_vel_nav=obstacle_avoidance.cmd_vel_nav:main',
            'path=obstacle_avoidance.pat:main',
	    'haptic = obstacle_avoidance.vibrotactile_feedback:main',
	    'haptic_http_node = obstacle_avoidance.haptic_http_node:main',
        ],
    },
)
