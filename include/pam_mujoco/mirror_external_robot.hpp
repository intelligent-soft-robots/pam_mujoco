#pragma once

#include "o80/memory_clearing.hpp"
#include "o80/back_end.hpp"
#include "o80/state2d.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{

  template<int QUEUE_SIZE, int NB_DOFS>
  class MirrorExternalRobot : public ControllerBase
  {
    
  private:
    typedef o80::BackEnd<QUEUE_SIZE,
			 NB_DOFS,
			 o80::State2d,
			 o80::VoidExtendedState> Backend;
    typedef o80::States<NB_DOFS,o80::State2d> States;

  public:
    MirrorExternalRobot(std::string segment_id,
			const mjModel* m,
			const mjData* d_init);
    void set_state(mjData* d);
    void apply(const mjModel* m,
		 mjData* d);
  public:
    static void clear(std::string segment_id);
  private:
    int index_q_robot_;
    int index_qvel_robot_;
    Backend backend_;
    States states_;
    
  };

#include "mirror_external_robot.hxx"
  
}
