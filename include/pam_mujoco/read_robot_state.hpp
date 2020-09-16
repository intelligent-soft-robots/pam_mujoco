#pragma once

#include "shared_memory/array.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{

  class ReadRobotState 
  {
  public:
    ReadRobotState(std::string segment_id);
    const std::array<double,4>& get_positions();
    const std::array<double,4>& get_velocities();
  private:
    shared_memory::array<JointState> joint_states_;
    std::string segment_id_;
    std::array<double,4> positions_;
    std::array<double,4> velocities_;
  };

}
