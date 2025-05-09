from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明参数
    can_interface = LaunchConfiguration('can_interface', default='can0')
    sync_period_ns = LaunchConfiguration('sync_period_ns', default='1000000')  # 1kHz
    iir_alpha = LaunchConfiguration('iir_alpha', default='0.05')
    use_hw_timestamp = LaunchConfiguration('use_hw_timestamp', default='false')
    
    # 获取配置文件路径
    pkg_dir = get_package_share_directory('yz_motor_driver')
    motor1_config = os.path.join(pkg_dir, 'config', 'motor1.yaml')
    motor2_config = os.path.join(pkg_dir, 'config', 'motor2.yaml')
    motor3_config = os.path.join(pkg_dir, 'config', 'motor3.yaml')
    motor4_config = os.path.join(pkg_dir, 'config', 'motor4.yaml')
    
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN接口名称'),
        
        DeclareLaunchArgument(
            'sync_period_ns',
            default_value='1000000',
            description='SYNC周期 (纳秒)'),
        
        DeclareLaunchArgument(
            'iir_alpha',
            default_value='0.05',
            description='IIR滤波系数 (0-1)'),
            
        DeclareLaunchArgument(
            'use_hw_timestamp',
            default_value='false',
            description='是否使用硬件时间戳 (MCP251x不支持)'),
        
        # SYNC主时钟节点
        Node(
            package='yz_motor_driver',
            executable='sync_master_node',
            name='sync_master_node',
            parameters=[{
                'can_interface': can_interface,
                'sync_period_ns': sync_period_ns,
                'iir_alpha': iir_alpha,
                'use_hw_timestamp': use_hw_timestamp,
            }],
            output='screen',
        ),
        
        # 电机1节点
        Node(
            package='yz_motor_driver',
            executable='yz_motor_node',
            name='motor1',
            parameters=[motor1_config],
            output='screen',
            remappings=[
                ('/position_deg_cmd', '/motor1_cmd_deg'),
                ('/position_deg', '/motor1_pos_deg'),
                ('/status', '/motor1_status'),
            ],
        ),
        
        # 电机2节点
        Node(
            package='yz_motor_driver',
            executable='yz_motor_node',
            name='motor2',
            parameters=[motor2_config],
            output='screen',
            remappings=[
                ('/position_deg_cmd', '/motor2_cmd_deg'),
                ('/position_deg', '/motor2_pos_deg'),
                ('/status', '/motor2_status'),
            ],
        ),
        
        # 电机3节点
        Node(
            package='yz_motor_driver',
            executable='yz_motor_node',
            name='motor3',
            parameters=[motor3_config],
            output='screen',
            remappings=[
                ('/position_deg_cmd', '/motor3_cmd_deg'),
                ('/position_deg', '/motor3_pos_deg'),
                ('/status', '/motor3_status'),
            ],
        ),
        
        # 电机4节点
        Node(
            package='yz_motor_driver',
            executable='yz_motor_node',
            name='motor4',
            parameters=[motor4_config],
            output='screen',
            remappings=[
                ('/position_deg_cmd', '/motor4_cmd_deg'),
                ('/position_deg', '/motor4_pos_deg'),
                ('/status', '/motor4_status'),
            ],
        ),
    ])