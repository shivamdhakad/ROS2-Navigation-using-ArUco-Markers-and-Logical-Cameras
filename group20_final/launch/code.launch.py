from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the parameters YAML file
    params_file_path = os.path.join(get_package_share_directory('group20_final'), 'config', 'waypoint_params.yaml')

    # Create the node with parameters loaded from the YAML file
    cameraImageCallback = Node(
        package='group20_final',
        executable='code',
        parameters=[params_file_path]
    )

    return LaunchDescription([cameraImageCallback])
    
    
   
