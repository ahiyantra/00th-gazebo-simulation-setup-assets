
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription,  TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Normally, we'd use get_package_share_directory here, but since our package isn't properly installed, we'll use a workaround.
    """
    pkg_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    """
    pkg_name = 'package-01'
    pkg_share = get_package_share_directory(pkg_name)
    """
    # Gazebo Ignition launch
    world_file = os.path.join(pkg_share, 'worlds', 'demo-01.sdf')
    ign_gz = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )
    # ROS2-Gazebo Ignition bridge
    """
    Normally, we'd use an executable file as a node here, but since bridge bugs when launched as a node, we'll use a workaround.
    """
    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )
    """
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
    )
    """
    # RViz2
    """
    Normally, we'd use an executable file as a node here, but since rviz crashes when launched as a node, we'll use a workaround.
    """
    rviz_config_file = os.path.join(pkg_share, 'config', 'rs-general-igngz.rviz')
    rviz2 = ExecuteProcess(
        cmd=[
                'rviz2',
                '-d', str(rviz_config_file)
            ],
        output='screen'
    )
    """
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    """
    # Launch Description
    return LaunchDescription([
        ign_gz,
        TimerAction(period=10.0, actions=[bridge]), 
        TimerAction(period=20.0, actions=[rviz2]),  
    ])
