cmake_minimum_required(VERSION 3.8)
project(humbot_description)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY meshes urdf launch rviz models worlds photos
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
