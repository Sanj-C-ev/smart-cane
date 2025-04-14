from setuptools import find_packages, setup

package_name = 'gps_waypoint'

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
    maintainer_email='ssanjeev17537@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_destination_input = gps_waypoint.manual_destination_input:main',
            'gps_waypoint_pub = gps_waypoint.waypoint_pubisher:main',
        ],
    },
)
