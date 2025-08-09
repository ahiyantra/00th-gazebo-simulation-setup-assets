# rviz_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rviz_config_file = os.path.join(pkg_share, 'config', 'rs-general-igngz.rviz')

    rviz2 = ExecuteProcess(
        cmd=[
            'x-terminal-emulator', '-e', 'ros2 run rviz2 rviz2 -d ' + rviz_config_file
        ],
        output='screen'
    )

    return LaunchDescription([
        rviz2
    ])
