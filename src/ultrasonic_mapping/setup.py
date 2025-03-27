from setuptools import find_packages, setup

package_name = 'ultrasonic_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'serial_ultrasonic = ultrasonic_mapping.serial_data:main',
            'us_mapping = ultrasonic_mapping.us_occupancy_map:main',
            'occupancy_map_2 = ultrasonic_mapping.occupancy_map_2:main'
        ],
    },
)
