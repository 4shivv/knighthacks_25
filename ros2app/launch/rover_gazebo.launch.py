from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_simulation')
    world_file = os.path.join(pkg_dir, 'worlds', 'rover_world.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'rover.urdf.xacro')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                         'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover', '-file', urdf_file],
        output='screen'
    )
    
    rover_controller = Node(
        package='rover_simulation',
        executable='rover_controller',
        name='rover_controller',
        output='screen'
    )
    
    obstacle_detector = Node(
        package='rover_simulation',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen'
    )
    
    vision_processor = Node(
        package='rover_simulation',
        executable='vision_processor',
        name='vision_processor',
        output='screen'
    )
    
    flask_bridge = Node(
        package='rover_simulation',
        executable='flask_bridge',
        name='flask_bridge',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity,
        rover_controller,
        obstacle_detector,
        vision_processor,
        flask_bridge
    ])