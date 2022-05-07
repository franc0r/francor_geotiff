import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='francor_geotiff',
            executable='geotiff_node',
            name='francor_geotiff_node',
            output='screen',
            prefix='nice -n 15',            
            parameters=[
              {"map_file_path": os.path.join(get_package_share_directory('francor_geotiff'), '/maps')},
              {"map_file_base_name": "test_hans"},
              {"geotiff_save_period": 0.0},
              {"VictimMapWriter/draw_all_objects": False},
              {"VictimMapWriter/class_id": "none"},
              {"QRCodeMapWriter/draw_all_objects": False},
              {"QRCodeMapWriter/class_id": "none"},
            ]
        )
    ])