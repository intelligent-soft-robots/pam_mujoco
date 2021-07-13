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
find_package(pybind11 REQUIRED)
find_package(mujoco_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)

ament_export_dependencies(
  Boost
  o80
  o80_example
  json_helper
  pam_interface
  pam_models
  o80_pam
  real_time_tools
  shared_memory
  signal_handler
  mujoco_interface
  pybind11)

# prepare to export all needed targets
set(all_targets)
set(all_target_exports)


#######################
# Configuration files #
#######################

get_filename_component(MODEL_CONFIG_DIR models ABSOLUTE)
# installing files in /opt/mpi-is/ if exists
if (EXISTS "/opt/mpi-is")
  install(DIRECTORY
    "${MODEL_CONFIG_DIR}" DESTINATION /opt/mpi-is/)
endif()
# installing in workspace/install
install(DIRECTORY
  "${MODEL_CONFIG_DIR}" DESTINATION opt/mpi-is/)
set(MODELS_PATH "/tmp/")


###########
# Library #
###########

add_library( ${PROJECT_NAME} SHARED
  src/read_robot_state.cpp
  src/contact_ball.cpp
  src/contact_states.cpp
  src/contact_logic.cpp
  src/recompute_state_after_contact.cpp
  src/controllers.cpp
  src/add_controllers.cpp
  src/listener.cpp
  src/mujoco_config.cpp
  src/logger.cpp
  src/listener.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${mujoco_include_dir}
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  PAM_DEFAULT_CONFIG_FILE_PATH="${JSON_DEFAULT_CONFIG_FILE}")
target_link_libraries(${PROJECT_NAME} context::context)
target_link_libraries(${PROJECT_NAME} shared_memory::shared_memory)
target_link_libraries(${PROJECT_NAME} pam_models::pam_models)
target_link_libraries(${PROJECT_NAME} o80_pam::o80_pam)
target_link_libraries(${PROJECT_NAME} mujoco_interface::mujoco_interface)
foreach(mujoco_lib ${mujoco_libs})
  target_link_libraries(${PROJECT_NAME} ${mujoco_lib_dir}/${mujoco_lib})
endforeach(mujoco_lib)
target_link_libraries(${PROJECT_NAME} stdc++fs)
target_link_libraries(${PROJECT_NAME} EGL GL GLU glut)
target_compile_definitions(pam_mujoco PRIVATE
  MODEL_PATHS="${MODELS_PATH}")
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})

add_executable( launch_pam_mujoco
  src/pam_mujoco.cpp )
target_include_directories(
  launch_pam_mujoco PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${mujoco_include_dir}
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(launch_pam_mujoco PRIVATE
  PAM_DEFAULT_CONFIG_FILE_PATH="${JSON_DEFAULT_CONFIG_FILE}")
target_link_libraries(launch_pam_mujoco shared_memory::shared_memory)
target_link_libraries(launch_pam_mujoco mujoco_interface::mujoco_interface)
target_link_libraries(launch_pam_mujoco o80::o80)
target_link_libraries(launch_pam_mujoco context::context)
target_link_libraries(launch_pam_mujoco ${PROJECT_NAME})
foreach(mujoco_lib ${mujoco_libs})
  target_link_libraries(launch_pam_mujoco ${mujoco_lib_dir}/${mujoco_lib})
endforeach(mujoco_lib)
target_link_libraries(launch_pam_mujoco stdc++fs)
target_link_libraries(launch_pam_mujoco EGL GL GLU glut)
list(APPEND all_targets launch_pam_mujoco)


###################
# Python wrappers #
###################

add_library(${PROJECT_NAME}_py MODULE srcpy/wrappers.cpp)
target_compile_definitions(${PROJECT_NAME}_py PRIVATE
  PAM_DEFAULT_CONFIG_FILE_PATH="${JSON_DEFAULT_CONFIG_FILE}")
target_link_libraries(${PROJECT_NAME}_py PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_py
  PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}"
  OUTPUT_NAME ${PROJECT_NAME}_wrp)
target_include_directories(
  ${PROJECT_NAME}_py
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
target_compile_definitions(pam_mujoco PRIVATE
  MODEL_PATHS="${MODELS_PATH}")
_ament_cmake_python_get_python_install_dir()
install(TARGETS ${PROJECT_NAME}_py DESTINATION ${PYTHON_INSTALL_DIR})


######################
# Python Native code #
######################

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})


###############
# Executables #
###############

configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mujoco
  ${CMAKE_INSTALL_PREFIX}/bin/pam_mujoco COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mujoco_no_xterms
  ${CMAKE_INSTALL_PREFIX}/bin/pam_mujoco_no_xterms COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mujoco_stop
  ${CMAKE_INSTALL_PREFIX}/bin/pam_mujoco_stop COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mujoco_stop_all
  ${CMAKE_INSTALL_PREFIX}/bin/pam_mujoco_stop_all COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mujoco_visualization
  ${CMAKE_INSTALL_PREFIX}/bin/pam_mujoco_visualization COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mujoco_visualization_no_xterms
  ${CMAKE_INSTALL_PREFIX}/bin/pam_mujoco_visualization_no_xterms COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/process_pam_visualization
  ${CMAKE_INSTALL_PREFIX}/bin/process_pam_visualization COPYONLY)


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
