cmake_minimum_required(VERSION 3.8)
project(face_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)

# 添加可执行文件
# 目标名称是 face_detector_node，源文件是 src/face_detector_node.cpp
add_executable(face_detector_node 
  src/face_detector_node.cpp
  src/face_detector.cpp
)


target_include_directories(face_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



# 链接依赖项
# ament_target_dependencies 会自动处理 include 目录和链接库
ament_target_dependencies(face_detector_node
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
  image_transport
  OpenCV

)


# 安装规则
install(TARGETS
  face_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME} 
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME} 
)
install(DIRECTORY moudles
  DESTINATION share/${PROJECT_NAME} 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()