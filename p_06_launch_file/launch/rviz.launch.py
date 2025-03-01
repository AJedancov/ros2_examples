from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    urdf_tutorial_path = FindPackageShare('p_06_launch_file')
    default_rviz_config_path = PathJoinSubstitution([urdf_tutorial_path, 'rviz', 'config.rviz'])

    rviz_config_launch_arg = DeclareLaunchArgument(
        name='rviz_config', 
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    ld.add_action(rviz_config_launch_arg)
    ld.add_action(rviz_node)

    return ld