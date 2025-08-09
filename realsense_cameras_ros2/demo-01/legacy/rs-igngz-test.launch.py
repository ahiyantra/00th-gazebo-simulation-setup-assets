
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Ignition Gazebo
    gz_sim = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '../worlds/demo-01.sdf'
        ],
        output='screen'
    )

    # RViz 2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', '../config/rs-igngz-test.rviz'
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

    # Launch arguments
    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Open RViz 2.'
            ),
        bridge,
        rviz,
    ])
