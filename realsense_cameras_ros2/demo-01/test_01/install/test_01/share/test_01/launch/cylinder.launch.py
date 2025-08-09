import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
#from launch import LaunchDescription
#from launch_ros.actions import Node

def generate_launch_description():
    #pkgPath = launch_ros.substitutions.FindPackageShare(package='urdf_test').find('urdf_test')
    #urdfModelPath= os.path.join(pkgPath, 'urdf/model.urdf')
    pkgPath = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdfModelPath= os.path.join('..', 'urdf/cylinder.urdf')
    
    with open(urdfModelPath,'r') as infp:
    	robot_desc = infp.read()
    
    params = {'robot_description': robot_desc}
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
	    executable='robot_state_publisher',
        output='screen',
        parameters=[params]#, {'frame_prefix': ''}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        name='joint_state_publisher',
        parameters=[params],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    """
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    """
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    tf_node = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'base_link']
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='This is a flag for joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdfModelPath,
                                            description='Path to the urdf model file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        rviz_node,
        tf_node
    ]) 