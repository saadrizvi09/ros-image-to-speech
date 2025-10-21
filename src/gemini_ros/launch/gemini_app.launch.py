import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- Define the nodes ---

    # Gemini Vision Node with parameter
    vision_node = Node(
        package='gemini_ros',
        executable='vision_node',
        name='gemini_vision_node',
        output='screen',
        parameters=[{
            'image_path': '/home/saad-rizvi/Pictures/Screenshots/a.png'
            # IMPORTANT: Change this to your actual image path
        }]
    )

    # Speech Input Node
    speech_node = Node(
        package='gemini_ros',
        executable='speech_node',
        name='speech_input_node',
        output='screen'
    )

    # --- Create the launch description and add the nodes ---
    return LaunchDescription([
        vision_node,
        speech_node
    ])