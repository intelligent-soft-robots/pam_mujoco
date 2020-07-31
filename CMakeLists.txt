cmake_minimum_required(VERSION 2.8.7)

project(pam_mujoco)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  mujoco200
  pam_models
  mpi_cmake_modules
)

catkin_package(
 LIBRARIES pam_models
 INCLUDE_DIRS include
 CATKIN_DEPENDS mujoco200 pam_models mpi_cmake_modules
)

DEFINE_OS()

include_directories(include ${catkin_INCLUDE_DIRS})



