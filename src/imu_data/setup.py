from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'imu_data'

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
            'imu_data = imu_data.imu_serial_data:main',
            'sensor_data = imu_data.data_collection:main'
        ],
    },
)
