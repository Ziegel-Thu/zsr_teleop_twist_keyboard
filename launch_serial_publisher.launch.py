#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='串口设备路径'
        ),
        
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='串口波特率'
        ),
        
        DeclareLaunchArgument(
            'timeout',
            default_value='1.0',
            description='串口超时时间(秒)'
        ),
        
        DeclareLaunchArgument(
            'topic_name',
            default_value='zsr_cmd_vel',
            description='订阅的话题名称'
        ),
        
        DeclareLaunchArgument(
            'data_format',
            default_value='binary',
            description='数据格式: binary 或 text'
        ),
        
        DeclareLaunchArgument(
            'use_crc',
            default_value='true',
            description='是否使用CRC校验'
        ),
        
        # 启动串口发布器节点
        Node(
            package='zsr_teleop_twist_keyboard',
            executable='zsr_serial_publisher',
            name='zsr_serial_publisher',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'timeout': LaunchConfiguration('timeout'),
                'topic_name': LaunchConfiguration('topic_name'),
                'data_format': LaunchConfiguration('data_format'),
                'use_crc': LaunchConfiguration('use_crc'),
            }],
            remappings=[
                ('zsr_cmd_vel', LaunchConfiguration('topic_name')),
            ]
        )
    ]) 