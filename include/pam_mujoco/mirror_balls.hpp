#pragma once

#include <map>
#include "o80/memory_clearing.hpp"
#include "o80/back_end.hpp"
#include "o80/state1d.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/joint_state.hpp"
#include "pam_mujoco/contacts.hpp"

namespace pam_mujoco
{

  template<int QUEUE_SIZE, int NB_BALLS>
  class MirrorBalls : public ControllerBase
  {
    
  private:
    typedef o80::BackEnd<QUEUE_SIZE,
			 NB_BALLS*6, // 6: 3d position and 3d velocity per ball
			 o80::State1d,
			 o80::VoidExtendedState> Backend;
    typedef o80::States<NB_BALLS*6,o80::State1d> States;

  public:
    MirrorBalls(std::string segment_id,
		std::string ball_obj_joint);
    MirrorBalls(std::string segment_id,
		std::string ball_obj_joint,
		const std::map<int,
		               std::string>& ball_index_segment_id);
    void set_contact_interrupt(int ball_index,
			       std::string segment_id);
    void set_contact_interrupt(const std::map<int,std::string>& ball_index_segment_id);
    void set_state(mjData* d);
    void apply(const mjModel* m,
		 mjData* d);
  public:
    static void clear(std::string segment_id);
  private:
    std::string ball_obj_joint_;
    int index_q_balls_;
    int index_qvel_balls_;
    Backend backend_;
    States states_;
    std::array<bool,NB_BALLS> contact_interrupts_;
    std::array<bool,NB_BALLS> interrupted_;
    std::array<std::string,NB_BALLS> segment_id_contacts_;
    
  };

#include "mirror_balls.hxx"
  
}
