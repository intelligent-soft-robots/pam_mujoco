// Copyright (c) 2020 Max Planck Gesellschaft
// Author : Vincent Berenz

#include "shared_memory/serializer.hpp"
#include "o80/pybind11_helper.hpp"
#include "o80/state1d.hpp"
#include "o80/state2d.hpp"
#include "o80/void_extended_state.hpp"
#include "pam_mujoco/run_mujoco.hpp"

#define NB_DOFS 4
#define QUEUE_SIZE 500000



PYBIND11_MODULE(pam_mujoco, m)
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
    (m,std::string("MirrorOneBall"));

  pybind11::class_<pam_mujoco::RecomputeStateConfig>(m,"RecomputeStateConfig")
    .def(pybind11::init<>())
    .def_readwrite("epsilon_r",&pam_mujoco::RecomputeStateConfig::epsilon_r)
    .def_readwrite("epsilon_t_x",&pam_mujoco::RecomputeStateConfig::epsilon_t_x)
    .def_readwrite("epsilon_t_z",&pam_mujoco::RecomputeStateConfig::epsilon_t_z)
    .def_readwrite("y_vel_plus",&pam_mujoco::RecomputeStateConfig::y_vel_plus)
    .def_readwrite("z_vel_plus",&pam_mujoco::RecomputeStateConfig::z_vel_plus)
    .def_readwrite("rot_matrix_contactee_zero_pos",
		   &pam_mujoco::RecomputeStateConfig::rot_matrix_contactee_zero_pos);

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
  
  m.def("init_mujoco",&pam_mujoco::init_mujoco);
  m.def("add_mirror_robot",&pam_mujoco::add_mirror_robot);
  m.def("add_mirror_one_ball_robot",&pam_mujoco::add_mirror_one_ball);
  m.def("add_contact_ball",&pam_mujoco::add_default_contact_ball);
  m.def("add_bursting",&pam_mujoco::add_bursting);
  m.def("execute",&pam_mujoco::execute);
  m.def("request_stop",&pam_mujoco::request_stop);
  m.def("is_stop_requested",&pam_mujoco::is_stop_requested);
}
