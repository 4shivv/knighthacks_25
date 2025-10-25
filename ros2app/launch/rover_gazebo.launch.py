from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_simulation')
    world_file = os.path.join(pkg_dir, 'worlds', 'rover_world.sdf')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'rover.urdf.xacro')
    
    # Start Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-file', urdf_file,
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )
    
    # Bridge for cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    
    # Bridge for odometry
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )
    
    # Bridge for LiDAR
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )
    
    # Bridge for camera
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )
    
    # ROS2 nodes
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
        gz_sim,
        spawn_entity,
        bridge_cmd_vel,
        bridge_odom,
        bridge_lidar,
        bridge_camera,
        rover_controller,
        obstacle_detector,
        vision_processor,
        flask_bridge
    ])