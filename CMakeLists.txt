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
  o80_pam
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
set(MODELS_PATH "${MODEL_CONFIG_DIR}/tmp/")

add_library(pam_mujoco
  src/contact_ball.cpp
  src/contact_states.cpp
  src/contact_logic.cpp
  src/contact_information.cpp
  src/recompute_state_after_contact.cpp
  src/burst_controller.cpp
  src/run_management.cpp
  src/run_mujoco.cpp
  src/controllers.cpp )
target_link_libraries(pam_mujoco ${catkin_LIBRARIES} stdc++fs)
target_compile_definitions(pam_mujoco PRIVATE
  MODEL_PATHS="${MODELS_PATH}")

pybind11_add_module(pam_mujoco_py srcpy/wrappers.cpp)
target_link_libraries(pam_mujoco_py PRIVATE pam_mujoco ${catkin_LIBRARIES})
set_target_properties(pam_mujoco_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
  OUTPUT_NAME pam_mujoco_wrp
  )
install(TARGETS pam_mujoco_py DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
target_compile_definitions(pam_mujoco_py PRIVATE
  MODEL_PATHS="${MODELS_PATH}")

# python package (will be extended with pam_mujoco_wrp)
catkin_python_setup()

build_sphinx_documentation()
