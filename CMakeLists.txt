cmake_minimum_required(VERSION 2.8.7)

project(pam_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

find_package(catkin REQUIRED COMPONENTS
  real_time_tools
  o80
  context
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

# absolute path to the models folder
get_filename_component(MODEL_CONFIG_DIR models ABSOLUTE)
set(MODELS_PATH "${MODEL_CONFIG_DIR}/")


add_executable(mujoco_simulate_example
  src/run_management.cpp
  src/mujoco_simulate_example.cpp)
target_link_libraries(mujoco_simulate_example
  ${CMAKE_THREAD_LIBS_INIT}
  ${catkin_LIBRARIES})

add_library(pam_mujoco
  src/contacts.cpp
  src/burst_controller.cpp
  src/run_management.cpp
  src/run_mujoco.cpp
  src/force_computes.cpp
  src/controllers.cpp )
target_link_libraries(pam_mujoco ${catkin_LIBRARIES})
target_compile_definitions(pam_mujoco PRIVATE
  MODEL_PATHS="${MODELS_PATH}")


add_executable(test_run
  src/test_run.cpp)
target_link_libraries(test_run
  pam_mujoco
  ${CMAKE_THREAD_LIBS_INIT}
  ${catkin_LIBRARIES})
target_compile_definitions(test_run PRIVATE
  MODEL_PATHS="${MODELS_PATH}")


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
target_compile_definitions(pam_mujoco_py PRIVATE
  MODEL_PATHS="${MODELS_PATH}")

catkin_add_gtest(pam_mujoco_tests
  tests/main.cpp
  tests/unit_tests.cpp)
target_link_libraries(pam_mujoco_tests pam_mujoco ${catkin_libraries})

