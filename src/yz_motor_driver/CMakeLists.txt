cmake_minimum_required(VERSION 3.8)
project(yz_motor_driver)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 移除不需要的依赖
# find_package(example_interfaces REQUIRED)

# 包含目录
include_directories(include)

# 添加库
add_library(canopen_driver src/canopen_driver.cpp)
add_library(cia402_driver src/cia402_driver.cpp)

# 添加带宽规划器库
add_library(can_bandwidth_planner src/can_bandwidth_planner.cpp)
target_link_libraries(can_bandwidth_planner)

# 添加带宽规划器节点
add_executable(bandwidth_planner_node src/bandwidth_planner_node.cpp)
target_link_libraries(bandwidth_planner_node can_bandwidth_planner)
ament_target_dependencies(bandwidth_planner_node rclcpp std_msgs std_srvs)

# 添加可执行文件
add_executable(yz_motor_node src/yz_motor_node.cpp)
target_link_libraries(yz_motor_node canopen_driver cia402_driver)
ament_target_dependencies(yz_motor_node rclcpp std_msgs std_srvs)

# 添加SYNC主时钟库和节点
add_library(sync_master src/sync_master.cpp)
ament_target_dependencies(sync_master rclcpp std_msgs)
target_link_libraries(sync_master pthread rt)

add_executable(sync_master_node src/sync_master_node.cpp)
target_link_libraries(sync_master_node sync_master)
ament_target_dependencies(sync_master_node rclcpp std_msgs std_srvs)

# 生成服务接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPositionReachedParams.srv"
  DEPENDENCIES std_msgs
)

# 确保依赖关系正确
ament_export_dependencies(rosidl_default_runtime)

# 安装目标
install(TARGETS
  canopen_driver
  cia402_driver
  yz_motor_node
  can_bandwidth_planner
  bandwidth_planner_node
  sync_master
  sync_master_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
