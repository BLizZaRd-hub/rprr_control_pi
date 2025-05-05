from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明参数
    can_interface = LaunchConfiguration('can_interface', default='can0')
    sync_period_ns = LaunchConfiguration('sync_period_ns', default='1000000')
    iir_alpha = LaunchConfiguration('iir_alpha', default='0.05')
    use_hw_timestamp = LaunchConfiguration('use_hw_timestamp', default='false')  # 默认禁用硬件时间戳
    
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
            default_value='false',  # 默认禁用硬件时间戳
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
    ])
