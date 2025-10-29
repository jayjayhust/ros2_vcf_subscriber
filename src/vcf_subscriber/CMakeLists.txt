cmake_minimum_required(VERSION 3.8)
project(vcf_subscriber)

# 检查C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(vcf_subscriber
  src/vcf_subscriber.cpp
)

# 添加依赖
ament_target_dependencies(vcf_subscriber
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# 链接OpenCV库
target_link_libraries(vcf_subscriber ${OpenCV_LIBS})

# 安装目标
install(TARGETS
  vcf_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# 如果确实需要安装launch文件，取消注释以下部分
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# 导出依赖
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

ament_package()