from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sidebyside_stereo_publisher',
            executable='sidebyside_stereo_publisher',
            name='sidebyside_stereo_publisher',
            output='screen',
            parameters=[
                {'publish_range_start': '0/4'},
                {'publish_range_end': '4/4'},
                {'video_dir': '/mnt/usb-in-container/roadside_mf/roadside_mf_0'},
#                {'video_dir': '/mnt/usb-in-container/roadside_ab/roadside_ab_0'},
#                {'video_dir': '/mnt/usb-in-container/apart_entrance_mf/apart_entrance_mf_0'},
#                {'video_dir': '/mnt/usb-in-container/apart_entrance_ab/apart_entrance_ab_0'},
#                {'video_dir': '/mnt/usb-in-container/apart_parkinglot_mf/apart_parkinglot_mf_0'},
#                {'video_dir': '/mnt/usb-in-container/apart_parkinglot_ab/apart_parkinglot_ab_0'},
#                {'video_dir': '/mnt/usb-in-container/yoyogi_forest1_mf/yoyogi_forest1_mf_0'},
#                {'video_dir': '/mnt/usb-in-container/yoyogi_forest1_ab/yoyogi_forest1_ab_0'},
#                {'video_dir': '/mnt/usb-in-container/yoyogi_forest2_mf/yoyogi_forest2_mf_0'},
#                {'video_dir': '/mnt/usb-in-container/yoyogi_forest2_ab/yoyogi_forest2_ab_0'},
#                {'video_dir': '/mnt/usb-in-container/yoyogi_heigen_mf/yoyogi_heigen_mf_0'},
#                {'video_dir': '/mnt/usb-in-container/yoyogi_heigen_ab/yoyogi_heigen_ab_0'},
                {'baseline': 0.065},
                {'img_size_x': 512},
                {'img_size_y': 512},
                {'fov_h': 90},
                {'fov_v': 90},
                {'fps_timer': 30.0},
                {'fps_timestamp': 30.0}
            ]
        )
    ])
