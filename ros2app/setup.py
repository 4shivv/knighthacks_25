from setuptools import setup

package_name = 'rover_simulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='user@example.com',
    description='ROS2 rover simulation with D* Lite planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_controller = rover_simulation.rover_controller:main',
            'obstacle_detector = rover_simulation.obstacle_detector:main',
            'vision_processor = rover_simulation.vision_processor:main',
            'flask_bridge = rover_simulation.flask_bridge:main',
        ],
    },
)