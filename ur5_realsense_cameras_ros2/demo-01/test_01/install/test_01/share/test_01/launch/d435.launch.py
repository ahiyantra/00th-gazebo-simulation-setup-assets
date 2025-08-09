import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare the launch arguments
    use_xacro = LaunchConfiguration('use_xacro')
    urdf_file = LaunchConfiguration('urdf_file')
    xacro_file = LaunchConfiguration('xacro_file')

    # Specify the path to your package
    #pkg_name = 'your_package_name'
    #pkg_path = get_package_share_directory(pkg_name)
    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    mesh_path = os.path.join(pkg_path, 'mesh')

    # Set up the URDF file paths
    default_urdf_file = os.path.join(pkg_path, 'urdf', '_d435.urdf')
    default_xacro_file = os.path.join(pkg_path, 'urdf', 'test_d435_camera.urdf.xacro') # _d435.urdf.xacro

    # Declare the launch arguments
    """
    declare_use_xacro = DeclareLaunchArgument(
        'use_xacro',
        default_value='true', # false
        description='Use xacro file instead of urdf'
    )
    declare_urdf_file = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf_file,
        description='Path to URDF file'
    )
    """
    declare_xacro_file = DeclareLaunchArgument(
        'xacro_file',
        default_value=default_xacro_file,
        description='Path to Xacro file'
    )

    # Choose between URDF and Xacro
    """
    robot_description = Command([
        'xacro ' if use_xacro == 'true' else 'cat ',
        xacro_file if use_xacro == 'true' else urdf_file
    ])
    """
    # Process the Xacro file
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('xacro_file'), ' mesh_path:=', mesh_path]),
                                       value_type=str)

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Joint State Publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # Joint State Publisher GUI node
    """
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    """
    # RViz2 node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'urdf_config.rviz')],
    )

    # Published Transform Frame
    tf2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'base_link']
    )

    # Create and return the launch description
    return LaunchDescription([
        #declare_use_xacro,
        #declare_urdf_file,
        declare_xacro_file,
        robot_state_publisher,
        joint_state_publisher,
        #joint_state_publisher_gui,
        rviz2,
        tf2
    ])