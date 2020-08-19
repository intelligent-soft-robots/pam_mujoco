cmake_minimum_required(VERSION 2.8.7)

project(pam_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

find_package(catkin REQUIRED COMPONENTS
  real_time_tools
  o80
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
 CATKIN_DEPENDS mujoco200 pam_models mpi_cmake_modules real_time_tools
)

DEFINE_OS()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS})

add_executable(mujoco_simulate_example
  src/run_management.cpp
  src/mujoco_simulate_example.cpp)
target_link_libraries(mujoco_simulate_example
  ${CMAKE_THREAD_LIBS_INIT}
  ${catkin_LIBRARIES})

add_library(pam_mujoco
  src/burst_controller.cpp
  src/run_management.cpp
  src/run_mujoco.cpp
  src/force_computes.cpp
  src/controllers.cpp
  src/pid.cpp)
target_link_libraries(pam_mujoco ${catkin_LIBRARIES})

#add_executable(pam_mujoco_mirror_robot
#  src/mirror_robot.cpp)
#target_link_libraries(pam_mujoco_mirror_robot pam_mujoco ${catkin_LIBRARIES})

pybind11_add_module(pam_mujoco_py srcpy/wrappers.cpp)
target_link_libraries(pam_mujoco_py PRIVATE pam_mujoco ${catkin_LIBRARIES})
set_target_properties(pam_mujoco_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
  OUTPUT_NAME pam_mujoco
  )
install(TARGETS pam_mujoco_py DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

catkin_add_gtest(pam_mujoco_tests
  tests/main.cpp
  tests/unit_tests.cpp)
target_link_libraries(pam_mujoco_tests pam_mujoco ${catkin_libraries})

