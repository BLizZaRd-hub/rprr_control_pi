from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明参数
    can_interface = LaunchConfiguration('can_interface', default='can0')
    node_id = LaunchConfiguration('node_id', default='1')
    position_scale = LaunchConfiguration('position_scale', default='32768.0')
    velocity_scale = LaunchConfiguration('velocity_scale', default='10.0')
    
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name'),
        
        DeclareLaunchArgument(
            'node_id',
            default_value='1',
            description='CANopen node ID'),
        
        DeclareLaunchArgument(
            'position_scale',
            default_value='32768.0',
            description='Position scale (encoder pulses per revolution)'),
        
        DeclareLaunchArgument(
            'velocity_scale',
            default_value='10.0',
            description='Velocity scale (rpm / scale)'),
        
        # 节点
        Node(
            package='yz_motor_driver',
            executable='yz_motor_node',
            name='yz_motor_node',
            parameters=[{
                'can_interface': can_interface,
                'node_id': node_id,
                'position_scale': position_scale,
                'velocity_scale': velocity_scale,
            }],
            output='screen'
        )
    ])
