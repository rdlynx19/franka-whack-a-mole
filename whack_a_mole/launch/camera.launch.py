"""
Launch Description Script for RealSense and AprilTag Detection

This script defines a launch file for initializing a RealSense camera, configuring 
its parameters, and setting up related nodes for point cloud processing and AprilTag 
detection. It includes the following key components:

1. **Launch Arguments**:
   - `depth_module.profile`: Specifies the resolution and framerate for the depth module (default: `1280x720x30`).
   - `rgb_camera.profile`: Specifies the resolution and framerate for the RGB camera (default: `1280x720x30`).
   - `align_depth`: Enables alignment of the depth image to the RGB camera frame (default: `true`).
   - `pointcloud.enable`: Toggles the generation of point clouds (default: `true`).

2. **Included Launch File**:
   - Launches the RealSense2 camera node using the `rs_launch.py` file from the `realsense2_camera` package.

3. **Nodes**:
   - **Rviz2**: Provides visualization of the detected tags or point cloud (optional; configuration file `tags_tf.rviz`).
   - **Table Node**: Processes point cloud data with remapping for depth-color points.
   - **AprilTag Node**: Detects AprilTags in the camera feed using the `apriltag_ros` package.
   - **AprilTags Detection Node**: A custom node for further processing of detected AprilTags.

4. **Configurations**:
   - Relies on a `tags.yaml` configuration file for AprilTag parameters.
   - Uses specific Rviz configuration files for visualizing the point cloud and AprilTags.

To run the launch file, use the following command:

"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare(package='whack_a_mole').find('whack_a_mole')
    tags_yaml = os.path.join(pkg_share, 'tags.yaml')
    tags_tf_rviz = os.path.join(pkg_share, 'tags_tf.rviz')
    move_it_rviz = os.path.join(pkg_share, 'moveit.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_only",
            default_value="false",
            description="Launch camera only."
        ),
        DeclareLaunchArgument(
            "align_depth",
            default_value="true",
            description="Align depth to RGB camera frame."
        ),
        DeclareLaunchArgument(
            "pointcloud.enable",
            default_value="true",
            description="Enable point cloud generation."
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare('realsense2_camera'),
                        "launch",
                        "rs_launch.py"
                    ]
                )
            ),
            launch_arguments={
                "depth_module.profile": "1280x720x30",
                "rgb_camera.profile": "1280x720x30",
                "enable_sync": "true",
                'align_depth.enable': "true",
                'enable_color': "true",
                'enable_depth': "true"
            }.items()
        ),

        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("camera_only"), "false")),
            package="rviz2",
            executable="rviz2",
            arguments=[
                '-d', move_it_rviz,
                '--ros-args', '--log-level', 'fatal'
            ],
            output="screen",
        ),
        Node(
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("camera_only"), "true")),
            package="rviz2",
            executable="rviz2",
            arguments=[
                '-d', tags_tf_rviz
            ],
            output="screen",
        ),
        Node(
            package='whack_a_mole',
            executable="camera_node",
            name = "color_camera",
            output = "screen",
            parameters=[ParameterFile(
                PathJoinSubstitution([pkg_share, 'params.yaml'])
            )]
        ),
    ])