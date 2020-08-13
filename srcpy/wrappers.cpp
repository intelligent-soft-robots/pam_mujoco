// Copyright (c) 2020 Max Planck Gesellschaft
// Author : Vincent Berenz

#include "o80/pybind11_helper.hpp"
#include "o80/state2d.hpp"
#include "o80/void_extended_state.hpp"

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
    (m);
}
