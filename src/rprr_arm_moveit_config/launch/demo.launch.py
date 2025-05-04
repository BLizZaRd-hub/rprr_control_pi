import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import pprint # 用于漂亮地打印字典

# OpaqueFunction to inspect the config and launch nodes
def launch_setup(context: LaunchContext, *args, **kwargs):
    print("--- Setting up MoveIt Launch ---")

    moveit_config_package = LaunchConfiguration("moveit_config_package").perform(context)
    rviz_config_file_name = LaunchConfiguration("rviz_config_file").perform(context)

    # 构建 RViz 文件路径字符串
    rviz_config_file_path_str = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", rviz_config_file_name]
    ).perform(context)
    print(f"RViz config path: {rviz_config_file_path_str}")

    try:
        # 构建 MoveItConfigs 对象
        moveit_config = (
            MoveItConfigsBuilder(robot_name="rprr_arm", package_name=moveit_config_package)
            .robot_description_semantic(file_path="config/rprr_arm.srdf")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .to_moveit_configs()
        )

        # 定义节点列表
        nodes_to_start = []
        
        # 1. Robot State Publisher
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        )
        nodes_to_start.append(robot_state_publisher_node)
        
        # 2. Joint State Publisher GUI
        joint_state_publisher_gui_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
        )
        nodes_to_start.append(joint_state_publisher_gui_node)
        
        # 3. Move Group 节点 - 这是解决规划问题的关键
        run_move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        )
        nodes_to_start.append(run_move_group_node)
        
        # 4. Static TF Publisher
        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        )
        nodes_to_start.append(static_tf)
        
        # 5. RViz 节点
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",  # 改为screen以便查看错误
            arguments=["-d", rviz_config_file_path_str],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.trajectory_execution,
            ],
        )
        nodes_to_start.append(rviz_node)
        
        # 6. Fake Controller Manager - 检查包是否可用
        try:
            from ament_index_python.packages import get_package_share_directory
            # 尝试获取包路径，如果包不存在会抛出异常
            get_package_share_directory('moveit_fake_controller_manager')
            
            # 如果没有抛出异常，说明包存在，添加节点
            fake_controller_manager_node = Node(
                package="moveit_fake_controller_manager",
                executable="moveit_fake_controller_manager",
                name="moveit_fake_controller_manager",
                parameters=[
                    moveit_config.trajectory_execution,
                    {"use_sim_time": True},
                ],
                output="screen",
            )
            nodes_to_start.append(fake_controller_manager_node)
            print("--- Added Fake Controller Manager ---")
        except Exception as e:
            print(f"!!! Warning: moveit_fake_controller_manager not found, skipping: {e}")
            print("!!! You may need to install it with: sudo apt install ros-humble-moveit-fake-controller-manager")

        print("--- MoveIt Launch Setup Complete ---")
        return nodes_to_start

    except Exception as e:
        print(f"!!! Error during MoveIt setup: {e}")
        import traceback
        traceback.print_exc()
        return []

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("moveit_config_package", default_value="rprr_arm_moveit_config"))
    declared_arguments.append(DeclareLaunchArgument("rviz_config_file", default_value="moveit.rviz"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])