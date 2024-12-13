import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare(package='whack_a_mole').find('whack_a_mole')
    tags_tf_rviz = os.path.join(pkg_share, 'tags_tf.rviz')
    move_it_rviz = os.path.join(pkg_share, 'moveit.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_only',
            default_value='false',
            description='Launch camera only.'
        ),
        DeclareLaunchArgument(
            'align_depth',
            default_value='true',
            description='Align depth to RGB camera frame.'
        ),
        DeclareLaunchArgument(
            'pointcloud.enable',
            default_value='true',
            description='Enable point cloud generation.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    ]
                )
            ),
            launch_arguments={
                'depth_module.profile': '1280x720x30',
                'rgb_camera.profile': '1280x720x30',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                'enable_color': 'true',
                'enable_depth': 'true'
            }.items()
        ),

        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('camera_only'), 'false')),
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', move_it_rviz,
                '--ros-args', '--log-level', 'fatal'
            ],
            output='screen',
        ),
        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('camera_only'), 'true')),
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', tags_tf_rviz
            ],
            output='screen',
        ),
        Node(
            package='whack_a_mole',
            executable='camera_node',
            name='color_camera',
            output='screen',
            parameters=[ParameterFile(
                PathJoinSubstitution([pkg_share, 'params.yaml'])
            )]
        ),
    ])
