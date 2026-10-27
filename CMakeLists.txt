cmake_minimum_required(VERSION 3.16)
project(sot CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # optimized by default, the simulation runs much faster
endif()

find_package(pinocchio REQUIRED)   # also brings Eigen

add_executable(sot sot.cpp)
target_link_libraries(sot pinocchio::pinocchio)
