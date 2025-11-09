cmake_minimum_required(VERSION 3.8)
project(humbot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(twist_mux_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(safety_stop src/safety_stop.cpp)
ament_target_dependencies(safety_stop rclcpp rclcpp_action std_msgs sensor_msgs twist_mux_msgs visualization_msgs)

install(TARGETS
  safety_stop
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
