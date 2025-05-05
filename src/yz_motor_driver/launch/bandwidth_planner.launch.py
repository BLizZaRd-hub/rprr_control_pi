from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明参数
    bitrate = LaunchConfiguration('bitrate', default='1000000')
    safe_utilization = LaunchConfiguration('safe_utilization', default='0.30')
    num_axes = LaunchConfiguration('num_axes', default='4')
    sync_frequency = LaunchConfiguration('sync_frequency', default='1000.0')
    rpdo_dlc = LaunchConfiguration('rpdo_dlc', default='7')
    tpdo_dlc = LaunchConfiguration('tpdo_dlc', default='7')
    heartbeat_frequency = LaunchConfiguration('heartbeat_frequency', default='5.0')
    
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'bitrate',
            default_value='1000000',
            description='CAN总线比特率 (bit/s)'),
        
        DeclareLaunchArgument(
            'safe_utilization',
            default_value='0.30',
            description='安全利用率阈值 (0-1)'),
        
        DeclareLaunchArgument(
            'num_axes',
            default_value='4',
            description='电机轴数'),
        
        DeclareLaunchArgument(
            'sync_frequency',
            default_value='1000.0',
            description='SYNC频率 (Hz)'),
        
        DeclareLaunchArgument(
            'rpdo_dlc',
            default_value='7',
            description='RPDO数据长度'),
        
        DeclareLaunchArgument(
            'tpdo_dlc',
            default_value='7',
            description='TPDO数据长度'),
        
        DeclareLaunchArgument(
            'heartbeat_frequency',
            default_value='5.0',
            description='心跳频率 (Hz)'),
        
        # 带宽规划器节点
        Node(
            package='yz_motor_driver',
            executable='bandwidth_planner_node',
            name='bandwidth_planner_node',
            parameters=[{
                'bitrate': bitrate,
                'safe_utilization': safe_utilization,
                'num_axes': num_axes,
                'sync_frequency': sync_frequency,
                'rpdo_dlc': rpdo_dlc,
                'tpdo_dlc': tpdo_dlc,
                'heartbeat_frequency': heartbeat_frequency,
            }],
            output='screen',
        ),
    ])
