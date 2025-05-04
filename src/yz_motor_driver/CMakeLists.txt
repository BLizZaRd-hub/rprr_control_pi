cmake_minimum_required(VERSION 3.8)
project(yz_motor_driver)

# --- 查找依赖包 ---
# 查找 C++ 标准，推荐 C++17 或更高
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 ament_cmake，这是 ROS 2 CMake 构建的基础
find_package(ament_cmake REQUIRED)
# 查找 C++ 客户端库
find_package(rclcpp REQUIRED)
# 查找标准消息 (Float32, UInt16)
find_package(std_msgs REQUIRED)
# 查找标准服务 (Trigger)
find_package(std_srvs REQUIRED)
# 查找组件注册相关的包
find_package(rclcpp_components REQUIRED)

# --- 添加头文件目录 ---
include_directories(include)

# --- 创建库 ---
# 将 motor_driver.cpp 编译成一个库，方便节点链接
add_library(yz_motor_driver_lib src/motor_driver.cpp)
# 链接库依赖 (如果 motor_driver.cpp 内部直接用了 ROS 功能，比如日志)
target_link_libraries(yz_motor_driver_lib PRIVATE rclcpp)
# 设置目标属性，例如 C++ 标准
set_target_properties(yz_motor_driver_lib PROPERTIES
    CXX_STANDARD ${CMAKE_CXX_STANDARD}
    CXX_EXTENSIONS OFF)

# --- 创建节点 (作为可组合组件库) ---
# 将 yz_motor_driver_node.cpp 编译成一个共享库，用于组件加载
add_library(${PROJECT_NAME}_node SHARED
  src/yz_motor_driver_node.cpp
)
# 链接节点库到我们自己创建的驱动库和 ROS 2 库
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  # yz_motor_driver_lib # 链接到我们自己的库
)
# 使用 target_link_libraries 链接到本地定义的库 yz_motor_driver_lib
target_link_libraries(${PROJECT_NAME}_node PRIVATE yz_motor_driver_lib) # <--- 添加这行
# 设置目标属性
set_target_properties(${PROJECT_NAME}_node PROPERTIES
    CXX_STANDARD ${CMAKE_CXX_STANDARD}
    CXX_EXTENSIONS OFF)

# --- 注册节点为可加载组件 ---
rclcpp_components_register_nodes(${PROJECT_NAME}_node
  PLUGIN yz_motor_driver::YzMotorDriverNode
)
# 注意：这里的 "PLUGIN" 关键字很重要，表示这是一个可以被加载的节点插件。
# "yz_motor_driver::YzMotorDriverNode" 是我们在 C++ 代码中定义的、带命名空间的完整类名。

# --- 安装 ---
# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)
# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# 安装库文件 (yz_motor_driver_lib 和组件库)
install(TARGETS
  yz_motor_driver_lib
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin # 虽然是库，但有时也需要安装到 bin
)

# 安装 package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- Ament Lint (可选，代码规范检查) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Ament Package ---
# 必须在最后调用，完成包的配置
ament_package()