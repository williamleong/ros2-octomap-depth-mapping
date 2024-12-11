#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_prefixes', default_value="['back_left', 'back_right']"), # https://robotics.stackexchange.com/questions/94364/passing-array-of-strings-as-launch-arguments-in-ros2-python-launcher
        DeclareLaunchArgument('sensor_model/hit', default_value='0.7'),
        DeclareLaunchArgument('sensor_model/miss', default_value='0.4'),
        DeclareLaunchArgument('sensor_model/min', default_value='0.12'),
        DeclareLaunchArgument('sensor_model/max', default_value='0.97'),
        DeclareLaunchArgument('octomap/max_distance', default_value='20.0'),
        DeclareLaunchArgument('octomap/min_z', default_value='-2.0'),
        DeclareLaunchArgument('octomap/max_z', default_value='5.0'),
        DeclareLaunchArgument('encoding', default_value='mono8'),
        DeclareLaunchArgument('resolution', default_value='0.2'),
        DeclareLaunchArgument('frame_id', default_value='odom'),
        DeclareLaunchArgument('filename', default_value=''),
        DeclareLaunchArgument('save_on_shutdown', default_value='false'),
        Node(
            package='octomap_depth_mapping',
            executable='octomap_depth_mapping',
            name='octodm_node',
            # prefix=['gdbserver localhost:3000'],
            output='screen',
            remappings=[('map_out', 'octomap_fullmap')],
            # remappings=[('image_in', 'depth/rect'),
            #             ('pose_in', 'pose'),
            #             ('map_out', 'octomap_fullmap'),
            #             ('camerainfo_in', 'depth/camera_info')],
            parameters=[{'input_prefixes': LaunchConfiguration('input_prefixes'),
                         'resolution': LaunchConfiguration('resolution'),
                         'frame_id': LaunchConfiguration('frame_id'),
                         'sensor_model/hit': LaunchConfiguration('sensor_model/hit'),
                         'sensor_model/miss': LaunchConfiguration('sensor_model/miss'),
                         'sensor_model/min': LaunchConfiguration('sensor_model/min'),
                         'sensor_model/max': LaunchConfiguration('sensor_model/max'),
                         'octomap/max_distance': LaunchConfiguration('octomap/max_distance'),
                         'octomap/min_z': LaunchConfiguration('octomap/min_z'),
                         'octomap/max_z': LaunchConfiguration('octomap/max_z'),
                         'encoding': LaunchConfiguration('encoding'),
                         'filename': LaunchConfiguration('filename'),
                         'save_on_shutdown': LaunchConfiguration('save_on_shutdown')
                        }]
        )
    ])
