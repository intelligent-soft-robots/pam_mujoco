#pragma once

#include "o80/memory_clearing.hpp"
#include "o80/back_end.hpp"
#include "o80/state1d.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{

  template<int QUEUE_SIZE, int NB_BALLS>
  class MirrorBalls : public ControllerBase
  {
    
  private:
    typedef o80::BackEnd<QUEUE_SIZE,
			 NB_BALLS*6, // 3d position and 3d velocity per ball
			 o80::State1d,
			 o80::VoidExtendedState> Backend;
    typedef o80::States<NB_BALLS*6,o80::State1d> States;

  public:
    MirrorBalls(std::string segment_id);
    void set_state(mjData* d);
    void apply(const mjModel* m,
		 mjData* d);
  public:
    static void clear(std::string segment_id);
  private:
    int index_q_balls_;
    int index_qvel_balls_;
    Backend backend_;
    States states_;
    
  };

#include "mirror_balls.hxx"
  
}
