cmake_minimum_required(VERSION 3.8)
project(autoware_roi_points_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(tier4_perception_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(roi_points_fusion_node src/roi_points_fusion_node.cpp)
add_executable(roi_points_fusion_visualizer_node src/roi_points_fusion_visualizer_node.cpp)

target_include_directories(roi_points_fusion_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(roi_points_fusion_visualizer_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(roi_points_fusion_node
  rclcpp
  OpenCV
  message_filters
  tier4_perception_msgs
  sensor_msgs
  cv_bridge
)

ament_target_dependencies(roi_points_fusion_visualizer_node
  rclcpp
  OpenCV
  message_filters
  tier4_perception_msgs
  sensor_msgs
  cv_bridge
)

install(TARGETS
  roi_points_fusion_visualizer_node
  roi_points_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
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

add_compile_options(-Wno-unused-parameter)
ament_package()
