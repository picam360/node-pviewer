from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
import os

def generate_launch_description():
    """Launch file to bring up Visual SLAM and Robot Localization nodes."""

    display_rviz = DeclareLaunchArgument(
        'display_rviz', default_value='false', description='Flag to display RViz'
    )

    # Path to the EKF configuration file
    ekf_config_path = os.path.join(os.getcwd(), 'config', 'ekf_params.yaml')

    # Isaac ROS Visual SLAM node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('/stereo_camera/left/camera_info', '/camera_info_left'),
                    ('/stereo_camera/right/camera_info', '/camera_info_right')],
        parameters=[{
            'num_cameras': 2,
            'use_sim_time': False,
            'denoise_input_images': True,
            'rectified_images': False,
            #'rectified_images': True,
            #'enable_ground_constraint_in_odometry' : True,
            #'enable_ground_constraint_in_slam' : True,
            #'enable_imu_fusion': True,
            'imu_frame': 'imu_frame',
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'enable_debug_mode': False,
            'debug_dump_path': '/tmp/cuvslam',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'camera_optical_frames': [
                'front_stereo_camera_left_optical',
                'front_stereo_camera_right_optical'
            ],
            'base_frame': 'base_link'
        }]
    )

    # Visual SLAM container
    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    # Robot Localization EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[ekf_config_path],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', './rviz/vslam.cfg.rviz'],
        condition=IfCondition(LaunchConfiguration('display_rviz'))
    )

    return LaunchDescription([
        visual_slam_launch_container,
        #ekf_node,
        display_rviz,
        rviz_node
    ])
