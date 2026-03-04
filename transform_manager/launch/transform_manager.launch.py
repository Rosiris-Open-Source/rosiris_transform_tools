from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="transform_manager", 
            executable="transform_manager_node",    
            name="transform_manager",
            output="screen",
            parameters=[
                {
                    "dynamic_frames_publish_frequency" : 100.0
                }
            ],                        
            remappings=[],                        
        )
    ])
