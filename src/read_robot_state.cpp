#include "pam_mujoco/read_robot_state.hpp"

namespace pam_mujoco
{

  ReadRobotState::ReadRobotState(std::string segment_id,
				 int index_qpos,
				 int index_qvel)
    : joint_states_{segment_id,4,true,true}
  {}

  const std::array<4,double>& ReadRobotState::get_positions()
  {
    for(std::size_t dof=0;dof<4;dof++)
      {
	positions_[dof] = joint_states_.get(dof).get<0>();
      }
    return positions;
  }
  
  const std::array<4,double>& ReadRobotState::get_velocities()
  {
    for(std::size_t dof=0;dof<4;dof++)
      {
	velocities_[dof] = joint_states_.get(dof).get<1>();
      }
    return velocities;
  }
  
  
}
