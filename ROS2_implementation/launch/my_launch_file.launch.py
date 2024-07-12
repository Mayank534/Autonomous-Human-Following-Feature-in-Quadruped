from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_ros2_package',
            executable='object_detection_publisher',
            name='object_detection_publisher_node',
            output='screen',
            parameters=[{
                'param_name': 'param_value',  # Add any parameters you need here
            }]
        ),
        Node(
            package='my_ros2_package',
            executable='another_node',  # Add other nodes if needed
            name='another_node_name',
            output='screen'
        ),
    ])
