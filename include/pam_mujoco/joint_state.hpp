#pragma once

#include <tuple>
#include "o80/state.hpp"

namespace pam_mujoco
{
  class JointState : public o80::State<std::tuple<double,double>,
				       JointState>
  {
  public:
    JointState(){}
    JointState(std::tuple<double,double> position_velocity);
  };
}
