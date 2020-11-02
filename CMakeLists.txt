cmake_minimum_required(VERSION 3.10.2)

project(pam_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)


################
# Dependencies #
################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(shared_memory REQUIRED)
find_package(o80 REQUIRED)
find_package(o80_pam REQUIRED)
find_package(pam_models REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(json_helper REQUIRED)
find_package(context REQUIRED)
find_package(signal_handler REQUIRED)
find_package(pam_interface REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(mujoco REQUIRED)

ament_export_dependencies(
  o80
  o80_example
  json_helper
  pam_interface
  pam_models
  o80_pam
  real_time_tools
  shared_memory
  signal_handler
  pybind11
  mujoco)

# prepare to export all needed targets
set(all_targets)
set(all_target_exports)


#######################
# Configuration files #
#######################

get_filename_component(MODEL_CONFIG_DIR models ABSOLUTE)
install(DIRECTORY
  "${MODEL_CONFIG_DIR}" DESTINATION /opt/mpi-is/)
install(DIRECTORY
  "${MODEL_CONFIG_DIR}" DESTINATION opt/mpi-is/)
set(MODELS_PATH "opt/mpi-is/models/tmp/")


###########
# Library #
###########

add_library( ${PROJECT_NAME} SHARED
  src/read_robot_state.cpp
  src/contact_ball.cpp
  src/contact_states.cpp
  src/contact_logic.cpp
  src/recompute_state_after_contact.cpp
  src/burst_controller.cpp
  src/run_management.cpp
  src/run_mujoco.cpp
  src/controllers.cpp )
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} context::context)
target_link_libraries(${PROJECT_NAME} shared_memory::shared_memory)
target_link_libraries(${PROJECT_NAME} pam_models::pam_models)
target_link_libraries(${PROJECT_NAME} o80_pam::o80_pam)
target_link_libraries(${PROJECT_NAME} mujoco::mujoco200)
target_link_libraries(${PROJECT_NAME} mujoco::glew)
target_link_libraries(${PROJECT_NAME} mujoco::glewegl)
target_link_libraries(${PROJECT_NAME} mujoco::glfw)
target_link_libraries(${PROJECT_NAME} stdc++fs)
target_compile_definitions(pam_mujoco PRIVATE
  MODEL_PATHS="${MODELS_PATH}")
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})


###################
# Python wrappers #
###################

add_library(${PROJECT_NAME}_py MODULE srcpy/wrappers.cpp)
target_link_libraries(${PROJECT_NAME}_py PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_py
  PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}"
  OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(
  ${PROJECT_NAME}_py
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
target_compile_definitions(pam_mujoco PRIVATE
  MODEL_PATHS="${MODELS_PATH}")
_ament_cmake_python_get_python_install_dir()
install(TARGETS ${PROJECT_NAME}_py DESTINATION ${PYTHON_INSTALL_DIR})


###############
# Executables #
###############

configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_mujoco
  ${CMAKE_INSTALL_PREFIX}/bin/o80_mujoco COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mujoco
  ${CMAKE_INSTALL_PREFIX}/bin/o80_mujoco COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mirroring
  ${CMAKE_INSTALL_PREFIX}/bin/o80_mujoco COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_visualization
  ${CMAKE_INSTALL_PREFIX}/bin/o80_mujoco COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mujoco_mirroring
  ${CMAKE_INSTALL_PREFIX}/bin/o80_mujoco COPYONLY)


######################
# Install and export #
######################

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
ament_package()
