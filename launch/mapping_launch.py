#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_model/hit', default_value='0.7'),
        DeclareLaunchArgument('sensor_model/miss', default_value='0.4'),
        DeclareLaunchArgument('sensor_model/min', default_value='0.12'),
        DeclareLaunchArgument('sensor_model/max', default_value='0.97'),
        DeclareLaunchArgument('output/max_distance', default_value='5.0'),
        DeclareLaunchArgument('encoding', default_value='mono8'),
        DeclareLaunchArgument('resolution', default_value='0.1'),
        DeclareLaunchArgument('frame_id', default_value='world'),
        DeclareLaunchArgument('filename', default_value=''),
        DeclareLaunchArgument('save_on_shutdown', default_value='false'),
        Node(
            package='octomap_depth_mapping',
            executable='octomap_depth_mapping',
            name='octodm_node',
            # prefix=['gdbserver localhost:3000'],
            output='screen',
            remappings=[('image_in', 'depth/rect'),
                        ('pose_in', 'pose'),
                        ('map_out', 'octomap_fullmap'),
                        ('camerainfo_in', 'depth/camera_info')],
            parameters=[{'resolution': LaunchConfiguration('resolution'),
                         'frame_id': LaunchConfiguration('frame_id'),
                         'sensor_model/hit': LaunchConfiguration('sensor_model/hit'),
                         'sensor_model/miss': LaunchConfiguration('sensor_model/miss'),
                         'sensor_model/min': LaunchConfiguration('sensor_model/min'),
                         'sensor_model/max': LaunchConfiguration('sensor_model/max'),
                         'output/max_distance': LaunchConfiguration('output/max_distance'),
                         'encoding': LaunchConfiguration('encoding'),
                         'filename': LaunchConfiguration('filename'),
                         'save_on_shutdown': LaunchConfiguration('save_on_shutdown')
                        }]
        )
    ])
