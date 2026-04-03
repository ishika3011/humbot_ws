cmake_minimum_required(VERSION 3.8)
project(humbot_nmpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

#a-------------------cados path------------------------

set(ACADOS_SOURCE_DIR $ENV{ACADOS_SOURCE_DIR})

if(NOT ACADOS_SOURCE_DIR)
  message(FATAL_ERROR
    "ACADOS_SOURCE_DIR environment variable not set!\n"
    "Add this to ~/.bashrc:\n"
    " export ACADOS_SOURCE_DIR=$HOME/acados\n"
    "Then run: source ~/.bashrc"
  )
endif()

set(ACADOS_INCLUDE_DIR ${ACADOS_SOURCE_DIR}/include)
set(ACADOS_LIB_DIR ${ACADOS_SOURCE_DIR}/lib)

message(STATUS "acados include: ${ACADOS_INCLUDE_DIR}")
message(STATUS "acados lib: ${ACADOS_LIB_DIR}")

#----------generated c code path-----------------

set(SOLVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/nmpc/c_generated_code)

if(NOT EXISTS ${SOLVER_DIR})
  message(FATAL_ERROR
    "c_generated_code/ not found at ${SOLVER_DIR}\n"
    "Run this first:\n"
    " cd ~/humbot_ws/src/humbot_nmpc/nmpc\n"
    "This generates the C code that CMake needs to compile."
  )
  endif()

#-----------------build the plugin as shared library 

add_library(nmpc_controller SHARED
  src/nmpc_controller_node.cpp

  ${SOLVER_DIR}/acados_solver_unicycle.c
  ${SOLVER_DIR}/unicycle_model/unicycle_expl_ode_fun.c
  ${SOLVER_DIR}/unicycle_model/unicycle_expl_vde_forw.c
  ${SOLVER_DIR}/unicycle_model/unicycle_expl_vde_adj.c
)

#--------------------Include directories---------------
target_include_directories(nmpc_controller PUBLIC
  
  include
  
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/acados
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include

  ${SOLVER_DIR}
  ${SOLVER_DIR}/unicycle_model

)

#------link acados libraries
target_link_libraries(nmpc_controller
  ${ACADOS_LIB_DIR}/libacados.so
  ${ACADOS_LIB_DIR}/libhpipm.so
  ${ACADOS_LIB_DIR}/libblasfeo.so
  m       # libm.so      — math (sin, cos, sqrt)
)

#------------ros2 dependencies--------------

ament_target_dependencies(nmpc_controller
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  tf2
  tf2_ros
  pluginlib
)

#----------register Nav2 plugin---------------

pluginlib_export_plugin_description_file(nav2_core nmpc_plugin.xml)

# Install the compiled .so
install(TARGETS nmpc_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files 
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files 
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install plugin XML
install(FILES nmpc_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(nmpc_controller)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs
  nav2_core nav2_costmap_2d tf2 tf2_ros pluginlib
)



ament_package()
