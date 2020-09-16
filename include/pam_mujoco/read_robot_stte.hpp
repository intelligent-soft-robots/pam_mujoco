#pragma once

#include <map>
#include "o80/memory_clearing.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{

  class ReadRobotState : public ControllerBase
  {
  public:
    ReadRobotState(std::string segment_id);
    void apply(const mjModel* m,
	       mjData* d);
    
  public:
    static void clear(std::string segment_id);
    static std::array<4,JointState> read(segment_id);
  private:
    shared_memory::array<JointState> joint_states_;

    std::string segment_id_;
  };

}
