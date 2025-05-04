cmake_minimum_required(VERSION 3.22)
project(rprr_arm_moveit_config)

find_package(ament_cmake REQUIRED)
# 安装 config 目录下的所有文件
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 目录下的所有文件 (确保这行也在)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
