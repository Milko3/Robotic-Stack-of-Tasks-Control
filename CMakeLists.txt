cmake_minimum_required(VERSION 3.16)
project(sot CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pinocchio REQUIRED)   # also brings Eigen

add_executable(sot
  src/main.cpp        # Person C
  src/sot_core.cpp    # Person A
  src/robot.cpp       # Person B
  src/tasks.cpp       # Person B
)
target_include_directories(sot PRIVATE include)
target_link_libraries(sot pinocchio::pinocchio)
