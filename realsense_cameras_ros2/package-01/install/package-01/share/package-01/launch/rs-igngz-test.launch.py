
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = 'package-01'
    package_dir = get_package_share_directory(package_name)

    # Set the IGN_GAZEBO_RESOURCE_PATH variable
    models_path = os.path.join(package_dir, 'models')
    current_env = os.environ.copy()
    if 'IGN_GAZEBO_RESOURCE_PATH' in current_env:
        current_env['IGN_GAZEBO_RESOURCE_PATH'] += ':' + models_path
    else:
        current_env['IGN_GAZEBO_RESOURCE_PATH'] = models_path

    # Ignition Gazebo
    gz_sim = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', #'../worlds/demo-01_alt.sdf'
            PathJoinSubstitution([FindPackageShare(package_name), 'worlds', 'demo-01.sdf'])
        ],
        output='screen',
        additional_env=current_env
    )

    # RViz 2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', #'../config/rs-igngz-test.rviz'
            PathJoinSubstitution([FindPackageShare(package_name), 'config', 'rs-igngz-test.rviz'])

        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Parameter Bridge
    remappings = [('/camera', '/camera/image'),
                  ('/camera_info', '/camera/camera_info')]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen',
        remappings=remappings,
    )

    # Published Transform Frame
    tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'D435_openrobotics/d435_link/rgbd_camera']
    )

    # Launch Arguments
    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Open RViz 2.'
            ),
        bridge,
        rviz,
        tf
    ])
