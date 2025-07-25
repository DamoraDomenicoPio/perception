cmake_minimum_required(VERSION 3.8)
project(autoware_object_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ONNXRUNTIME_VERSION "1.20.1")
set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_utils REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(tier4_perception_msgs REQUIRED)



# find_library(ONNXRUNTIME_LIB onnxruntime
#   HINTS "${ONNXRUNTIME_ROOT}/lib"
#   REQUIRED)


set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)


add_executable(autoware_object_detector_node src/object_detector_node.cpp)
ament_target_dependencies(autoware_object_detector_node
  rclcpp
  OpenCV
  sensor_msgs
  autoware_utils
  autoware_perception_msgs
  tier4_perception_msgs
)
install(TARGETS
  autoware_object_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${ONNXRUNTIME_ROOT}/include
)

target_link_libraries(autoware_object_detector_node
  "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so.${ONNXRUNTIME_VERSION}"
)

set_target_properties(autoware_object_detector_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib:${ONNXRUNTIME_ROOT}/lib"
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# rclcpp_components_register_node(${PROJECT_NAME}_node
#   PLUGIN "autoware::object_detector::ObjectDetector"
#   EXECUTABLE ${PROJECT_NAME}_node_exe
# )


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




