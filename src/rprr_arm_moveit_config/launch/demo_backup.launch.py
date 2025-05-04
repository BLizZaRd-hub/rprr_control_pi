import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # --- 声明 Launch 参数 ---
    declared_arguments = []
    # description_package 参数可能不再直接需要，因为 builder 会自动查找
    # 但 description_file 仍然需要
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="rprr_arm.urdf.xacro",
            description="URDF/XACRO description file relative to description package.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="rprr_arm_moveit_config",
            description="MoveIt config package.",
        )
    )
    # Rviz config file
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="moveit.rviz", # 使用 moveit.rviz
            description="RViz configuration file",
        )
    )


    # --- 获取 Launch 配置值 ---
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    rviz_config_file= LaunchConfiguration("rviz_config_file")


    # --- 使用 MoveItConfigsBuilder (让它自动查找 URDF/Xacro) ---
    moveit_config = (
        MoveItConfigsBuilder(robot_name="rprr_arm", package_name=moveit_config_package)
        # *** 不再明确指定 robot_description 的 file_path 和 package ***
        # *** 让 builder 尝试在依赖包 (rprr_description) 中自动查找 ***
        # *** 它默认会找 config/rprr_arm.urdf.xacro 或 urdf/rprr_arm.urdf.xacro ***
        # *** 确保你的 xacro 文件在 rprr_description/urdf 目录下 ***
        .robot_description_semantic(file_path="config/rprr_arm.srdf") # 保持相对路径
        .trajectory_execution(file_path="config/moveit_controllers.yaml") # 保持相对路径
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .robot_description_kinematics(file_path="config/kinematics.yaml") # 保持相对路径
        # *** 添加 launch RViz 的配置 ***
        .launch_rviz(rviz_config_file=PathJoinSubstitution( # 构建 RViz 文件路径
            [FindPackageShare(moveit_config_package), "config", rviz_config_file]
        ))
        .to_moveit_configs()
    )

    # --- 定义核心节点 ---
    # Robot State Publisher (使用 builder 生成的参数)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description], # builder 应该能正确生成这个了
    )

    # Move Group 节点
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz (现在由 .launch_rviz() 处理，如果需要手动控制，保留之前的定义)
    # rviz_node = ... (如果 .launch_rviz() 不符合需求，可以手动定义)

    # Static TF Publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    # Joint State Publisher GUI
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )
    # MoveIt Fake Controller Manager
    fake_controller_manager_node = Node(
        package="moveit_fake_controller_manager",
        executable="moveit_fake_controller_manager",
        name="moveit_fake_controller_manager",
        parameters=[
             PathJoinSubstitution([FindPackageShare(moveit_config_package),"config","moveit_controllers.yaml"]),
        ],
        output="screen",
    )

    # --- 构建 Launch Description ---
    # 注意：如果使用了 .launch_rviz()，就不需要手动添加 rviz_node
    nodes_to_start = [
        robot_state_publisher_node,
        run_move_group_node,
        # rviz_node, # 可能由 .launch_rviz() 启动
        static_tf,
        joint_state_publisher_node,
        fake_controller_manager_node,
    ]
    # 获取由 .launch_rviz() 等方法生成的额外节点
    generated_nodes = moveit_config.trajectory_execution.get("nodes_to_start", []) # 示例，具体看 builder 文档
    generated_nodes.extend(moveit_config.planning_pipelines.get("nodes_to_start", []))
    generated_nodes.extend(moveit_config.planning_scene_monitor.get("nodes_to_start", [])) # 通常包含 RViz

    return LaunchDescription(declared_arguments + nodes_to_start + generated_nodes)