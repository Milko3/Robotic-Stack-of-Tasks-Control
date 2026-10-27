cmake_minimum_required(VERSION 3.16)
project(sot CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pinocchio (inkl. Eigen) och ament_index från ROS 2 Humble
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(sot sot.cpp)
target_link_libraries(sot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)
