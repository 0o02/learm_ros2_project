cmake_minimum_required(VERSION 3.8)
project(learm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 URDF、网格、配置文件、启动文件
install(
  DIRECTORY
    urdf
    meshes
    config
    launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
