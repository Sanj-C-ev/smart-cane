from setuptools import find_packages, setup

package_name = 'simple_navigation'

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
            'controller = simple_navigation.controller:main',
            'driver = simple_navigation.feedback_driver:main',
            'obstacle_avoidance = simple_navigation.ultrasonic_haptic:main',

        ],
    },
)
