import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_ros',
            executable='stereo',
            name='aruco_stereo_node',
            output='screen',
            parameters=[{
                'marker_size': 0.05,            # 打印的标签实际大小(m)
                'apriltag_decimate': 2.0,       # 30Hz 建议开启降采样
                'apriltag_threads': 4,          # 使用4线程
                'apriltag_refine_edges': True,  # 提高角点精度
                'camera_frame': 'left_camera_optical_frame',
            }],
            remappings=[
                # 将代码里的默认话题映射到你相机的实际话题
                ('/left/image_rect_color', '/your_camera/left/image_rect'),
                ('/right/image_rect_color', '/your_camera/right/image_rect'),
                ('/left/camera_info', '/your_camera/left/camera_info'),
                ('/right/camera_info', '/your_camera/right/camera_info'),
            ]
        )
    ])