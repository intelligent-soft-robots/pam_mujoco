#pragma once

#include <map>
#include "o80/memory_clearing.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{

  class ReadRobotState : 
  {
  public:
    ReadRobotState(std::string segment_id);
    const std::array<4,double>& get_positions();
    const std::array<4,double>& get_velocities();
  private:
    shared_memory::array<JointState> joint_states_;
    std::string segment_id_;
    std::array<4,double> positions_;
    std::array<4,double> velocities_;
  };

}
