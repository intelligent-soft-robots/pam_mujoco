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
find_package(fmt REQUIRED)

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


###########
# Library #
###########

add_library( ${PROJECT_NAME} SHARED
  src/read_robot_state.cpp
  src/contact_ball.cpp
  src/contact_states.cpp
  src/recompute_state_after_contact.cpp
  src/controllers.cpp
  src/add_controllers.cpp
  src/burster_controller.cpp
  src/listener.cpp
  src/mujoco_config.cpp
  src/is_in_contact.cpp
  src/listener.cpp
  src/mj_state_tools.cpp
  )
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${mujoco_include_dir}
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  PAMY1_DEFAULT_CONFIG_FILE_PATH="${PAMY1_JSON_DEFAULT_CONFIG_FILE}"
  PAMY2_DEFAULT_CONFIG_FILE_PATH="${PAMY2_JSON_DEFAULT_CONFIG_FILE}"
)
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
target_link_libraries(${PROJECT_NAME} fmt::fmt)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})

add_executable( launch_pam_mujoco
  src/pam_mujoco.cpp
  src/mj_state_tools.cpp
)
target_include_directories(
  launch_pam_mujoco PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${mujoco_include_dir}
  $<INSTALL_INTERFACE:include>)
target_link_libraries(launch_pam_mujoco shared_memory::shared_memory)
target_link_libraries(launch_pam_mujoco mujoco_interface::mujoco_interface)
target_link_libraries(launch_pam_mujoco o80::o80)
target_link_libraries(launch_pam_mujoco context::context)
target_link_libraries(launch_pam_mujoco fmt::fmt)
target_link_libraries(launch_pam_mujoco ${PROJECT_NAME})
foreach(mujoco_lib ${mujoco_libs})
  target_link_libraries(launch_pam_mujoco ${mujoco_lib_dir}/${mujoco_lib})
endforeach(mujoco_lib)
target_link_libraries(launch_pam_mujoco stdc++fs)
target_link_libraries(launch_pam_mujoco EGL GL GLU glut)
list(APPEND all_targets launch_pam_mujoco)

add_executable(pam_mujoco_load_snapshot
  src/pam_mujoco_load.cpp
  src/mj_state_tools.cpp
)
target_include_directories(pam_mujoco_load_snapshot
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${mujoco_include_dir}
    $<INSTALL_INTERFACE:include>)
target_compile_definitions(pam_mujoco_load_snapshot PRIVATE
  PAM_DEFAULT_CONFIG_FILE_PATH="${JSON_DEFAULT_CONFIG_FILE}")
target_link_libraries(pam_mujoco_load_snapshot shared_memory::shared_memory)
target_link_libraries(pam_mujoco_load_snapshot mujoco_interface::mujoco_interface)
target_link_libraries(pam_mujoco_load_snapshot o80::o80)
target_link_libraries(pam_mujoco_load_snapshot context::context)
target_link_libraries(pam_mujoco_load_snapshot fmt::fmt)
target_link_libraries(pam_mujoco_load_snapshot ${PROJECT_NAME})
foreach(mujoco_lib ${mujoco_libs})
  target_link_libraries(pam_mujoco_load_snapshot ${mujoco_lib_dir}/${mujoco_lib})
endforeach(mujoco_lib)
target_link_libraries(pam_mujoco_load_snapshot stdc++fs)
target_link_libraries(pam_mujoco_load_snapshot EGL GL GLU glut)
list(APPEND all_targets pam_mujoco_load_snapshot)

add_executable(mj_data_file_bin2json
  src/mj_data_file_bin2json.cpp
  src/mj_state_tools.cpp
)
target_include_directories(
  mj_data_file_bin2json PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${mujoco_include_dir}
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(launch_pam_mujoco PRIVATE
  PAM_DEFAULT_CONFIG_FILE_PATH="${JSON_DEFAULT_CONFIG_FILE}")
target_link_libraries(mj_data_file_bin2json fmt::fmt)
target_link_libraries(mj_data_file_bin2json ${PROJECT_NAME})
foreach(mujoco_lib ${mujoco_libs})
  target_link_libraries(mj_data_file_bin2json ${mujoco_lib_dir}/${mujoco_lib})
endforeach(mujoco_lib)
list(APPEND all_targets mj_data_file_bin2json)




###################
# Python wrappers #
###################

add_library(${PROJECT_NAME}_py MODULE srcpy/wrappers.cpp)
target_link_libraries(${PROJECT_NAME}_py PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_py PRIVATE fmt::fmt)
set_target_properties(${PROJECT_NAME}_py
  PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}"
  OUTPUT_NAME ${PROJECT_NAME}_wrp)
target_include_directories(
  ${PROJECT_NAME}_py
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
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
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/pam_mirroring_real_robot
  ${CMAKE_INSTALL_PREFIX}/bin/pam_mirroring_real_robot COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_robot_ball_replay
  ${CMAKE_INSTALL_PREFIX}/bin/o80_robot_ball_replay COPYONLY)


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

##############
# Unit tests #
##############

# ignored for now
#if(BUILD_TESTING)
#  find_package(ament_cmake_gtest REQUIRED)
#  ament_add_gtest(test_${PROJECT_NAME}_cpp tests/main.cpp tests/unittests.cpp)
#  target_link_libraries(test_${PROJECT_NAME}_cpp ${PROJECT_NAME})
#endif()
