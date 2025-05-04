import launch
from launch_ros.actions import ComposableNodeContainer # 用于加载组件
from launch_ros.descriptions import ComposableNode    # 描述要加载的组件
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generates the launch description for the YZ Motor Driver node."""

    # --- 声明启动参数 ---
    # 参数：CAN 接口名称
    declare_can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0', # 默认值
        description='Name of the CAN interface (e.g., can0)'
    )

    # 参数：CAN 比特率
    declare_can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value='500000', # 默认值
        description='CAN bus bitrate (e.g., 500000, 250000)'
    )

    # 参数：要控制的电机节点 ID 列表 (字符串形式，用逗号分隔)
    declare_node_ids_arg = DeclareLaunchArgument(
        'node_ids',
        default_value='3', # 默认只控制节点 3
        description='Comma-separated list of motor node IDs to control (e.g., "1,2,3,4")'
    )

    # --- 将字符串列表转换为整数列表 (在节点内部处理更佳，但 launch 文件也可以做简单转换) ---
    # 注意：直接在 launch 文件中进行复杂的类型转换不如在 C++ 节点中健壮
    # 这里我们还是将字符串直接传递给 C++ 节点，让节点内部解析
    # 如果需要更复杂的转换，可以考虑写一个小的 Python 脚本或使用 launch event handler

    # --- 配置要加载的组件 ---
    yz_driver_component = ComposableNode(
        package='yz_motor_driver',
        plugin='yz_motor_driver::YzMotorDriverNode', # 必须与 CMakeLists 中注册的插件名称完全一致
        name='yz_motor_driver_node', # 节点名称
        parameters=[{ # 将启动参数传递给节点
            'can_interface': LaunchConfiguration('can_interface'),
            'can_bitrate': LaunchConfiguration('can_bitrate'),
            # 将 node_ids 字符串解析为整数列表 (在 C++ 节点中完成)
            # 这里我们先尝试直接传递字符串，让 C++ 节点处理
            # 或者，如果 C++ 节点参数声明的是整数列表，需要用更复杂的方式转换
            # 暂时假设 C++ 节点能处理字符串或我们修改 C++ 节点参数类型
             'node_ids_str': LaunchConfiguration('node_ids') # 传递字符串形式
             # 注意：如果 C++ 节点参数是 std::vector<long int>，这里需要修改 C++ 代码
             # 来接收一个字符串参数 node_ids_str，然后在内部解析
             # 或者使用更高级的 launch 文件技巧来转换
        }],
        extra_arguments=[{'use_intra_process_comms': True}] # 启用进程内通信以提高效率
    )

    # --- 创建一个容器来运行组件 ---
    driver_container = ComposableNodeContainer(
        name='yz_driver_container',
        namespace='', # 可以设置命名空间
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            yz_driver_component # 将我们的组件添加到容器中
        ],
        output='screen', # 将容器的输出打印到屏幕
        emulate_tty=True, # 模拟 TTY，使日志颜色生效
    )

    # --- 返回 LaunchDescription 对象 ---
    return launch.LaunchDescription([
        declare_can_interface_arg,
        declare_can_bitrate_arg,
        declare_node_ids_arg,
        driver_container # 启动容器，容器会自动加载里面的组件
    ])