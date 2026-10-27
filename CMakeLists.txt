
cmake_minimum_required(VERSION 3.16)
project(sot LANGUAGES CXX)

find_package(pinocchio REQUIRED)

add_executable(sot sot.cpp)
target_compile_features(sot PRIVATE cxx_std_17)
target_link_libraries(sot PRIVATE pinocchio::pinocchio)
include(CTest)
if(BUILD_TESTING)
  add_executable(test_standard_sot test_standard_sot.cpp)
  target_compile_features(test_standard_sot PRIVATE cxx_std_17)
  target_link_libraries(test_standard_sot PRIVATE pinocchio::pinocchio)
  add_test(NAME standard_sot COMMAND test_standard_sot)
  add_test(NAME panda_kinematic_demo COMMAND sot "${CMAKE_CURRENT_SOURCE_DIR}/panda.urdf")
endif()
