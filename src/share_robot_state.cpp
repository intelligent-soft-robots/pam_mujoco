#include "pam_mujoco/share_robot_state.hpp"

namespace pam_mujoco
{

  ShareRobotState::ShareRobotState(std::string segment_id,
				 int index_qpos,
				 int index_qvel)
    : joint_states_{segment_id,4,true,true},
      index_qpos_(index_qpos),
      index_qvel_(index_qvel)
    {}
  
    void ShareRobotState::apply(const mjModel* m,
			       mjData* d)
    {
      // joint_states_ is a shared memory array,
      // so values directly shared
      for(std::size_t dof;dof<4;dof++)
	{
	  joint_states_[dof].set<0>(d->qpos[index_qpos_+dof]);
	  joint_states_[dof].set<1>(d->qvel[index_qvel_+dof]);
	}
    }
    
  void ShareRobotState::clear(std::string segment_id)
  {
    shared_memory::clear_array(segment_id);
  }

  
}
