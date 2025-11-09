cmake_minimum_required(VERSION 3.8)
project(humbot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

add_library(pd_motion_planner SHARED src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner ${dependencies})

add_library(pure_pursuit SHARED src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit ${dependencies})

pluginlib_export_plugin_description_file(nav2_core motion_planner_plugins.xml)

# add_executable(pd_motion_planner_algo src/pd_motion_planner_algo.cpp)
# ament_target_dependencies(pd_motion_planner_algo rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)

# add_executable(pure_pursuit_algo src/pure_pursuit_algo.cpp)
# ament_target_dependencies(pure_pursuit_algo rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)

install(
  DIRECTORY include
  DESTINATION include 
)

install(TARGETS
  # pd_motion_planner_algo
  # pure_pursuit_algo
  pd_motion_planner
  pure_pursuit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES motion_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(
  pd_motion_planner
  pure_pursuit
)
ament_package()
