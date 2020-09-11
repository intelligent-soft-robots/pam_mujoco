#pragma once

#include <map>
#include "o80/memory_clearing.hpp"
#include "o80/back_end.hpp"
#include "o80/state1d.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/joint_state.hpp"
#include "pam_mujoco/contact_information.hpp"

namespace pam_mujoco
{

  template<int QUEUE_SIZE>
  class MirrorFreeJoint : public ControllerBase
  {
    
  private:
    typedef o80::BackEnd<QUEUE_SIZE,
			 6, // 6: 3d position and 3d velocity per ball
			 o80::State1d,
			 o80::VoidExtendedState> Backend;
    typedef o80::States<6,o80::State1d> States;

  public:
    MirrorFreeJoint(std::string segment_id,
		    std::string joint,
		    int index_qpos,
		    int index_qvel);
    MirrorFreeJoint(std::string segment_id,
		    std::string joint,
		    int index_qpos,
		    int index_qvel,
		    std::string interrupt_segment_id);
    void set_contact_interrupt(std::string segment_id);
    void apply(const mjModel* m,
	       mjData* d);
    
  public:
    static void clear(std::string segment_id);
    
  private:
    std::string joint_;
    int index_qpos_;
    int index_qvel_;
    Backend backend_;
    States states_;
    bool contact_interrupt_;
    bool interrupted_;
    std::string segment_id_contact_;
    
  };

#include "mirror_free_joint.hxx"
  
}
