#include "pam_mujoco/logger.hpp"

namespace pam_mujoco
{

  Logger::Logger(size_t time_series_size,
		 std::string segment_id,
		 std::string robot_joint_base,
		 std::string ball_joint,
		 int ball_index_qpos,
		 int ball_index_qvel
	   )
    : segment_id_{segment_id},
	time_series_{segment_id,
		     time_series_size,
		     true},
      robot_joint_base_{robot_joint_base},
      index_q_robot_{-1},
      index_qvel_robot_{-1},
      ball_joint_{ball_joint},
      ball_index_qpos_{ball_index_qpos},
      ball_index_qvel_{ball_index_qvel}
    {}
  
  void Logger::apply(const mjModel* m, mjData* d)
    {
      
      if (this->must_update(d))
	{
	  if (index_q_robot_ < 0)
	    {
	      index_q_robot_ =
		m->jnt_qposadr[mj_name2id(
					  m, mjOBJ_JOINT, robot_joint_base_.c_str())];
	      index_qvel_robot_ =
		m->jnt_dofadr[mj_name2id(
					 m, mjOBJ_JOINT, robot_joint_base_.c_str())];
	    }
	  for(std::size_t dim=0;dim<3;dim++)
	    {
	      item_.ball.position[dim]=d->qpos[ball_index_qpos_+dim];
	      item_.ball.velocity[dim]=d->qvel[ball_index_qvel_+dim];
	    }
	  for(std::size_t dof=0;dof<4;dof++)
	    {
	      item_.robot.positions[dof]=d->qpos[index_q_robot_ + dof];
	      item_.robot.velocities[dof]=d->qvel[index_qvel_robot_ + dof];
	    }
	  item_.time_stamp=this->get_time_stamp().count();
	  time_series_.append(item_);
	}
    }
  
}
