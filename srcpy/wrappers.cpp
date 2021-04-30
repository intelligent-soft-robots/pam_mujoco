// Copyright (c) 2020 Max Planck Gesellschaft
// Author : Vincent Berenz

#include "shared_memory/serializer.hpp"
#include "o80/pybind11_helper.hpp"
#include "pam_mujoco/run_mujoco.hpp"
#include "pam_mujoco/read_robot_state.hpp"
#include "pam_mujoco/mujoco_config.hpp"

#define NB_DOFS 4
#define QUEUE_SIZE 500000



PYBIND11_MODULE(pam_mujoco_wrp, m)
{

  pybind11::class_<pam_mujoco::ReadRobotState>(m,"ReadRobotState")
    .def(pybind11::init<std::string>())
    .def("get_positions",&pam_mujoco::ReadRobotState::get_positions)
    .def("get_velocities",&pam_mujoco::ReadRobotState::get_velocities);

  pybind11::class_<pam_mujoco::Config>(m,"Config")
    .def(pybind11::init<std::string>())
    .def("set_burst_mode",&pam_mujoco::Config::set_burst_mode)
    .def("set_model_path",&pam_mujoco::Config::set_model_path)
    .def("set_accelerated_time",&pam_mujoco::Config::set_accelerated_time)
    .def("__str__",&pam_mujoco::Config::to_string);
  m.def("set_mujoco_config",&pam_mujoco::set_mujoco_config);
  

  m.def("get_contact",[](std::string segment_id)
	{
	  context::ContactInformation ci;
	  shared_memory::deserialize(segment_id,segment_id,ci);
	  return ci;
	});

  pybind11::class_<pam_mujoco::MujocoConfig>(m,"MujocoConfig")
    .def(pybind11::init<>())
    .def_readwrite("graphics",&pam_mujoco::MujocoConfig::graphics)
    .def_readwrite("extended_graphics",&pam_mujoco::MujocoConfig::extended_graphics)
    .def_readwrite("realtime",&pam_mujoco::MujocoConfig::realtime);
  
  m.def("init_mujoco",&pam_mujoco::init_mujoco);
  m.def("add_mirror_robot",&pam_mujoco::add_mirror_robot);
  m.def("add_share_robot_state",&pam_mujoco::add_share_robot_state);
  m.def("add_mirror_free_joint",&pam_mujoco::add_mirror_free_joint);
  m.def("add_mirror_until_contact_free_joint",
	&pam_mujoco::add_mirror_until_contact_free_joint);
  m.def("add_table_contact_free_joint",&pam_mujoco::add_table_contact_free_joint);
  m.def("add_robot1_contact_free_joint",&pam_mujoco::add_robot1_contact_free_joint);
  m.def("add_robot2_contact_free_joint",&pam_mujoco::add_robot2_contact_free_joint);
  m.def("add_pressure_controller",&pam_mujoco::add_4dofs_pressure_controller);
  m.def("add_bursting",&pam_mujoco::add_bursting);
  m.def("execute",&pam_mujoco::execute);
  m.def("request_stop",&pam_mujoco::request_stop);
  m.def("is_stop_requested",&pam_mujoco::is_stop_requested);
  m.def("has_mujoco_started",&pam_mujoco::has_mujoco_started);
  m.def("wait_for_mujoco",&pam_mujoco::wait_for_mujoco);
  m.def("clear",&pam_mujoco::clear);
  m.def("reset_contact",&pam_mujoco::reset_contact);
  m.def("activate_contact",&pam_mujoco::activate_contact);
  m.def("deactivate_contact",&pam_mujoco::deactivate_contact);
}
