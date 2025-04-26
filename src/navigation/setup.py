from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'navigation'

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
            'controller = navigation.controller:main',
            'haptic_driver = navigation.haptic_driver:main',
            'kinesthetic_driver = navigation.kinesthetic_driver:main',
            'final_navigation = navigation.combined_haptic_kinesthetic:main'           

        ],
    },
)
