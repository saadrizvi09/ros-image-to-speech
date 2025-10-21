import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    vision_node = Node(
        package='gemini_ros',
        executable='vision_node',
        name='gemini_vision_node',
        output='screen',
        parameters=[{
            'use_camera': True,  # Use webcam
            'camera_index': 0,
            'language': 'en'
        }]
    )

    speech_node = Node(
        package='gemini_ros',
        executable='speech_node',
        name='speech_input_node',
        output='screen'
    )

    return LaunchDescription([
        vision_node,
        speech_node
    ])