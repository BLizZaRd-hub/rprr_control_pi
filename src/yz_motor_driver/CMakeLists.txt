cmake_minimum_required(VERSION 3.8)
project(yz_motor_driver)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# 移除 find_package(example_interfaces REQUIRED)

# 包含目录
include_directories(include)

# 添加库
add_library(canopen_driver src/canopen_driver.cpp)
add_library(cia402_driver src/cia402_driver.cpp)

# 添加可执行文件
add_executable(yz_motor_node src/yz_motor_node.cpp)
target_link_libraries(yz_motor_node canopen_driver cia402_driver)
ament_target_dependencies(yz_motor_node rclcpp std_msgs std_srvs)

# 安装目标
install(TARGETS
  canopen_driver
  cia402_driver
  yz_motor_node
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

ament_package()
