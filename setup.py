from setuptools import find_packages, setup

package_name = 'ugv_pkg'

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
            "video_display_node = ugv_pkg.video_display_node:main"
        ],
    },
)
