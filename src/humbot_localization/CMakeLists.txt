cmake_minimum_required(VERSION 3.8)
project(humbot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include) #to include header files

add_executable(kalman_filter src/kalman_filter.cpp)
ament_target_dependencies(kalman_filter 
  rclcpp
  sensor_msgs
  nav_msgs
)

add_executable(imu_republisher src/imu_republisher.cpp)
ament_target_dependencies(imu_republisher 
  rclcpp
  sensor_msgs
)

add_executable(odometry_motion_model src/odometry_motion_model.cpp)
ament_target_dependencies(odometry_motion_model 
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

install(  #this is for hpp files
  DIRECTORY include
  DESTINATION include
)

install(TARGETS 
  kalman_filter
  imu_republisher
  odometry_motion_model
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
