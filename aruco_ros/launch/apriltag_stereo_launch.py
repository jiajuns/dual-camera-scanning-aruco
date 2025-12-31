import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    udp_ip_arg = DeclareLaunchArgument(
        'udp_ip', default_value='192.168.1.100',
        description='Target UDP IP address')
    
    udp_port_arg = DeclareLaunchArgument(
        'udp_port', default_value='8888',
        description='Target UDP port')

    udp_rate_arg = DeclareLaunchArgument(
        'udp_rate', default_value='15.0',
        description='UDP send rate in Hz')

    return LaunchDescription([
        udp_ip_arg,
        udp_port_arg,
        udp_rate_arg,
        Node(
            package='aruco_ros',
            executable='stereo',
            name='aruco_stereo_node',
            output='screen',
            parameters=[{
                'marker_size': 0.05,            # 打印的标签实际大小(m)
                'apriltag_decimate': 2.0,       # 30Hz 建议开启降采样
                'apriltag_threads': 1,          # 使用1线程
                'apriltag_refine_edges': True,  # 提高角点精度
                'camera_frame': 'left_camera_optical_frame',
                'udp_ip': LaunchConfiguration('udp_ip'),
                'udp_port': LaunchConfiguration('udp_port'),
                'udp_send_rate_hz': LaunchConfiguration('udp_rate'),
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
