from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明参数
    can_interface = LaunchConfiguration('can_interface', default='can0')

    # 获取配置文件路径
    pkg_dir = get_package_share_directory('yz_motor_driver')
    motor3_config = os.path.join(pkg_dir, 'config', 'motor3.yaml')
    motor4_config = os.path.join(pkg_dir, 'config', 'motor4.yaml')

    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN接口名称'),

        # 只启动电机3节点
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
                # 添加服务重映射
		        ('/enable', '/motor3/enable'),
		        ('/disable', '/motor3/disable'),
		        ('/position_mode', '/motor3/position_mode'),
            ],
        ),

        # 只启动电机4节点
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
			    # 添加服务重映射
		        ('/enable', '/motor4/enable'),
        		('/disable', '/motor4/disable'),
		        ('/position_mode', '/motor4/position_mode'),
    ],
        ),
    ])
