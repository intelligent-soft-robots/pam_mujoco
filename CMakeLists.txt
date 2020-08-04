cmake_minimum_required(VERSION 2.8.7)

project(pam_mujoco)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  mujoco200
  pam_models
  mpi_cmake_modules
  pam_interface
)

search_for_pthread()
search_for_eigen()

catkin_package(
 LIBRARIES pam_models
 INCLUDE_DIRS include
 CATKIN_DEPENDS mujoco200 pam_models mpi_cmake_modules pam_interface
)

DEFINE_OS()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS})

add_executable(mujoco_simulate_example
  src/mujoco_simulate_example.cpp)
target_link_libraries(mujoco_simulate_example
  ${CMAKE_THREAD_LIBS_INIT}
  ${catkin_LIBRARIES})

add_library(pam_mujoco
  src/pid.cpp)
target_link_libraries(pam_mujoco ${catkin_LIBRARIES})

catkin_add_gtest(pam_mujoco_tests
  tests/main.cpp
  tests/unit_tests.cpp)
target_link_libraries(pam_mujoco_tests pam_mujoco ${catkin_libraries})

