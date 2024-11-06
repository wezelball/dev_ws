from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ugv_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to include the launch directory
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dcohen',
    maintainer_email='wezelball@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_control_node = ugv_pkg.motor_control_node:main",
            "battery_service_node = ugv_pkg.battery_service_node:main",
            "video_stream_subscriber_node = ugv_pkg.video_stream_subscriber_node:main",
            "video_display_node = ugv_pkg.video_display_node:main",
            "gimbal_control_node = ugv_pkg.gimbal_control_node:main",
            "lights_control_node = ugv_pkg.lights_control_node:main",
            "joystick_gimbal_control = ugv_pkg.joystick_gimbal_control:main",
            "lidar_data_publisher = ugv_pkg.lidar_data_publisher:main"
        ],
    },
)
