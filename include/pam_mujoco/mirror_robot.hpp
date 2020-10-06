#pragma once

#include "o80/memory_clearing.hpp"
#include "o80/back_end.hpp"
#include "o80/state2d.hpp"
#include "o80/time.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{

  template<int QUEUE_SIZE, int NB_DOFS>
  class MirrorRobot : public ControllerBase
  {
    
  private:
    typedef o80::BackEnd<QUEUE_SIZE,
			 NB_DOFS,
			 o80::State2d,
			 o80::VoidExtendedState> Backend;
    typedef o80::States<NB_DOFS,o80::State2d> States;

  public:
    MirrorRobot(std::string segment_id,
		std::string robot_joint_base);
    void apply(const mjModel* m,
		 mjData* d);
  private:
    void set_state(mjData* d);
  public:
    static void clear(std::string segment_id);
  private:
    Backend backend_;
    std::string robot_joint_base_;
    int index_q_robot_;
    int index_qvel_robot_;
    States read_states_;
    States set_states_;
  };

#include "mirror_robot.hxx"
  
}
