# bridge_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    bridge = ExecuteProcess(
        cmd=[
            'x-terminal-emulator', '-e', 'ros2 run ros_gz_bridge parameter_bridge ' +
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image ' +
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo ' +
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image ' +
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo ' +
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image ' +
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    return LaunchDescription([
        bridge
    ])
