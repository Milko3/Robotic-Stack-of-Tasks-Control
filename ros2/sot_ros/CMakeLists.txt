cmake_minimum_required(VERSION 3.16)
project(sot_ros CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
set(DEPS
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  std_msgs
  visualization_msgs
  geometry_msgs
)
foreach(dep IN ITEMS ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(pinocchio REQUIRED)

# The SoT code is the repo's sot.cpp (two folders up), included by the controller.
set(SOT_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_library(sot_controller SHARED src/sot_controller.cpp)
target_include_directories(sot_controller PRIVATE ${SOT_REPO_ROOT})
ament_target_dependencies(sot_controller PUBLIC ${DEPS})
target_link_libraries(sot_controller PUBLIC pinocchio::pinocchio)
# Rebuild the controller when sot.cpp changes.
set_property(SOURCE src/sot_controller.cpp APPEND PROPERTY OBJECT_DEPENDS ${SOT_REPO_ROOT}/sot.cpp)

pluginlib_export_plugin_description_file(controller_interface sot_controller_plugin.xml)

install(TARGETS sot_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config launch urdf rviz meshes
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${DEPS})
ament_package()
