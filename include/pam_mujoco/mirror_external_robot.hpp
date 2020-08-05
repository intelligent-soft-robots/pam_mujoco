#pragma once

#include "o80/memory_clearing"
#include "o80/backend.hpp"
#include "pam_interface/state/robot.hpp"
#include "pam_interface/state/joint.hpp"
#include "pam_mujoco/mujoco_base.hpp"
#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{

  template<int CONTROLLER_INDEX,
	   int QUEUE_SIZE, int NB_DOFS>
  class MirrorExternalRobot : public ControllerBase
  {
    
  private:
    typedef o80::BackEnd<QUEUE_SIZE,
			 NB_DOFS,
			 pam_interface::JointState,
			 o80::VoidExtendedState> Backend;
    typedef o80::States<NB_DOFS,pam_interface::JointState> States;

  public:
    MirrorExternalRobot(std::string segment_id,
			     int index_q_robot,
			     int index_qvel_robot,
			     mjData* d_init);
    void set_state(mjData* d);
    void control(const mjModel* m,
		 mjData* d);
  public:
    static void clear(std::string segment_id);
  private:
    int index_q_robot_;
    int index_qvel_robot_;
    BackEnd backend_;
    States states_;
    
  };

#include "mirror_external_robot.hxx"
  
}
