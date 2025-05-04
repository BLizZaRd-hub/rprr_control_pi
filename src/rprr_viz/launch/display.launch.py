import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 rprr_description 包的安装路径
    rprr_description_pkg_dir = get_package_share_directory('rprr_description')
    # 获取 rprr_viz 包的安装路径 (如果需要加载 RViz 配置)
    rprr_viz_pkg_dir = get_package_share_directory('rprr_viz')

    # 定义 URDF/Xacro 文件路径
    default_model_path = os.path.join(rprr_description_pkg_dir, 'urdf/rprr_arm.urdf.xacro')
    # 定义 RViz 配置文件路径 (可选, 先注释掉)
    # default_rviz_config_path = os.path.join(rprr_viz_pkg_dir, 'config/default.rviz')

    # --- 定义启动参数 ---
    # 用于指定 URDF 文件的参数
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')

    # (可选) 用于指定 RViz 配置文件的参数
    # declare_rviz_config_cmd = DeclareLaunchArgument(
    #     name='rvizconfig',
    #     default_value=default_rviz_config_path,
    #     description='Absolute path to rviz config file')

    # --- 定义要启动的节点 ---
    # 1. robot_state_publisher: 读取 URDF 并发布 TF tree
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            # 使用 xacro 命令处理 URDF 文件，并将结果作为 robot_description 参数传递
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]),
                value_type=str
            )
        }]
    )

    # 2. joint_state_publisher_gui: 提供 GUI 来手动设置关节状态
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 3. rviz2: 启动 RViz 可视化工具
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # (可选) 加载 RViz 配置文件
        # arguments=['-d', LaunchConfiguration('rvizconfig')]
        output='screen' # 将 RViz 的输出打印到终端，方便调试
    )

    # --- 创建启动描述 ---
    ld = launch.LaunchDescription()

    # 添加启动参数
    ld.add_action(declare_model_path_cmd)
    # ld.add_action(declare_rviz_config_cmd) # 可选

    # 添加要启动的节点
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
