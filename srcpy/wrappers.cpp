// Copyright (c) 2020 Max Planck Gesellschaft
// Author : Vincent Berenz

#include "shared_memory/serializer.hpp"
#include "o80/pybind11_helper.hpp"
#include "o80/state1d.hpp"
#include "o80/state2d.hpp"
#include "o80/void_extended_state.hpp"
#include "pam_mujoco/run_mujoco.hpp"
#include "pam_mujoco/read_robot_state.hpp"

#define NB_DOFS 4
#define QUEUE_SIZE 500000



PYBIND11_MODULE(pam_mujoco_wrp, m)
{
  o80::create_python_bindings<QUEUE_SIZE,
			      NB_DOFS,
			      o80::State2d,
			      o80::VoidExtendedState,
			      o80::NO_STATE, // o80::State2d already binded in package o80
			      o80::NO_EXTENDED_STATE> // same
    (m,std::string("MirrorRobot"));
  
  
  // 6 : position3d + velocity3d
  o80::create_python_bindings<QUEUE_SIZE,
			      6,
			      o80::State1d,
			      o80::VoidExtendedState,
			      o80::NO_STATE, // o80::State2d already binded in package o80
			      o80::NO_EXTENDED_STATE> // same
    (m,std::string("MirrorFreeJoint"));

  pybind11::class_<pam_mujoco::ContactInformation>(m,"ContactInformation")
    .def(pybind11::init<>())
    .def_readonly("position",&pam_mujoco::ContactInformation::position)
    .def_readonly("contact_occured",&pam_mujoco::ContactInformation::contact_occured)
    .def_readonly("time_stamp",&pam_mujoco::ContactInformation::time_stamp)
    .def_readonly("minimal_distance",&pam_mujoco::ContactInformation::minimal_distance);

  m.def("get_contact",[](std::string segment_id)
	{
	  pam_mujoco::ContactInformation ci;
	  shared_memory::deserialize(segment_id,segment_id,ci);
	  return ci;
	});

  pybind11::class_<pam_mujoco::ReadRobotState>(m,"ReadRobotState")
    .def(pybind11::init<std::string>())
    .def("get_positions",&pam_mujoco::ReadRobotState::get_positions)
    .def("get_velocities",&pam_mujoco::ReadRobotState::get_velocities);
  
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
  m.def("wait_for_mujoco",&pam_mujoco::wait_for_mujoco);
  m.def("clear",&pam_mujoco::clear);
  m.def("reset_contact",&pam_mujoco::reset_contact);
  m.def("activate_contact",&pam_mujoco::activate_contact);
  m.def("deactivate_contact",&pam_mujoco::deactivate_contact);
}
