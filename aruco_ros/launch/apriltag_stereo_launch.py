import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Target Serial Port')
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Serial Baud Rate')

    udp_rate_arg = DeclareLaunchArgument(
        'udp_rate', default_value='0.0',
        description='UDP send rate in Hz (0.0 to disable limit)')

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
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
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'udp_send_rate_hz': LaunchConfiguration('udp_rate'),
            }],
            remappings=[
                # 将代码里的默认话题映射到你相机的实际话题
                ('/left/image_rect_color', '/camera/camera/infra1/image_rect_raw'),
                ('/right/image_rect_color', '/camera/camera/infra2/image_rect_raw'),
                ('/left/camera_info', '/camera/camera/infra1/camera_info'),
                ('/right/camera_info', '/camera/camera/infra2/camera_info'),
            ]
        )
    ])
