// Copyright (c) 2020 Max Planck Gesellschaft
// Author : Vincent Berenz

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


  m.def("init_mujoco",&pam_mujoco::init_mujoco);
  m.def("add_mirror_robot",&pam_mujoco::add_mirror_robot);
  m.def("add_mirror_one_ball_robot",&pam_mujoco::add_mirror_one_ball);
  m.def("add_bursting",&pam_mujoco::add_bursting);
  m.def("execute",&pam_mujoco::execute);
  m.def("get_mirror_robot_segment_id",
	&pam_mujoco::get_mirror_robot_segment_id);
  m.def("get_mirror_one_ball_segment_id",
	&pam_mujoco::get_mirror_one_ball_segment_id);
  m.def("request_stop",&pam_mujoco::request_stop);
  m.def("is_stop_requested",&pam_mujoco::is_stop_requested);
}
