# ignition_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_file = os.path.join(pkg_share, 'worlds', 'demo-01.sdf')

    ign_gz = ExecuteProcess(
        cmd=['x-terminal-emulator', '-e', 'ign gazebo ' + world_file],
        output='screen'
    )

    return LaunchDescription([
        ign_gz
    ])
