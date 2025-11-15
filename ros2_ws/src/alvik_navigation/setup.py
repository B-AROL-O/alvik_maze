from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'alvik_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kev',
    maintainer_email='kev@example.com',
    description='Alvik robot navigation and SLAM package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_receiver = alvik_navigation.tof_receiver:main',
            'odom_publisher = alvik_navigation.odom_publisher:main',
            'click_to_move = alvik_navigation.click_to_move:main',
            'teleop_keyboard = alvik_navigation.teleop_keyboard:main',
        ],
    },
)
