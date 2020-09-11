#include "pam_mujoco/recompute_state_after_contact.hpp"

namespace pam_mujoco
{

  RecomputeStateConfig get_table_recompute_config()
  {
    RecomputeStateConfig config;
    config.epsilon = {0.73,0.73,0.92};
    config.rot_matrix_contactee_zero_pos = {1.,0.,0.,
					    0.,1.,0.,
					    0.,0.,1.};
    config.vel_plus.fill(0.);
    config.mirror_y=false;
    return config;
  }

  
  RecomputeStateConfig get_robot1_recompute_config()
  {
    RecomputeStateConfig config;
    config.epsilon = {0.78,0.78,0.78};
    config.rot_matrix_contactee_zero_pos = {0.000, -1.000, 0.000,
					    -1.000, 0.000, 0.000,
					    0.000, 0.000, -1.000};
    config.vel_plus.fill(0.);
    config.mirror_y=true;
    return config;
  }

  
  RecomputeStateConfig get_robot2_recompute_config()
  {
    RecomputeStateConfig config;
    config.epsilon = {0.78,0.78,0.78};
    config.rot_matrix_contactee_zero_pos = {0.000, 1.000, 0.000,
					    1.000, 0.000, 0.000,
					    0.000, 0.000, 1.000};
    config.vel_plus.fill(0.);
    config.mirror_y=true;
    return config;
  }

  
  RecomputeStateConfig get_recompute_state_config(ContactItems item)
  {
    if (item==ContactItems::Robot1)
      return get_robot1_recompute_config();
    if (item==ContactItems::Robot2)
      return get_robot2_recompute_config();
    return get_table_recompute_config();
  }

  
  void recompute_state_after_contact(const RecomputeStateConfig& config,
				     const internal::ContactStates& pre_contact,
				     const internal::ContactStates& current,
				     double get_ball_position[3],
				     double get_ball_velocity[3])
  {

    double rot_matrix_contactee_rel[9];
    mju_mulMatMatT(rot_matrix_contactee_rel,
		   config.rot_matrix_contactee_zero_pos.data(),
		   pre_contact.contactee_orientation.data(),
		   3, 3, 3);

    double quat_rot[4];
    double quat_rot_neg[4];
    mju_mat2Quat(quat_rot, rot_matrix_contactee_rel);
    mju_negQuat(quat_rot_neg, quat_rot);

    double ball_pos_in_contactee_coord_sys[3];
    double ball_pos_trans[3];
    for(size_t i=0;i<3;i++)
      ball_pos_trans[i] = pre_contact.ball_position[i]-pre_contact.contactee_position[i];
    mju_rotVecQuat(ball_pos_in_contactee_coord_sys,
		   ball_pos_trans,
		   quat_rot);

    double ball_vel_in_contactee_coord_sys[3];
    mju_rotVecQuat(ball_vel_in_contactee_coord_sys,
		   pre_contact.ball_velocity.data(),
		   quat_rot);
      
    double contactee_vel_in_contactee_coord_sys[3];
    mju_rotVecQuat(contactee_vel_in_contactee_coord_sys,
		   current.contactee_velocity.data(),
		   quat_rot);

    // to do : check "until impact" logic
    double time_until_impact = -ball_pos_in_contactee_coord_sys[1] /
      (ball_vel_in_contactee_coord_sys[1] - contactee_vel_in_contactee_coord_sys[1]);
    double delta_t_step = current.time_stamp - pre_contact.time_stamp;
    double delta_t_after_impact = delta_t_step - time_until_impact;

    // to to: z axis symmetry for table
    double ball_vel_new_in_contactee_coord_sys[3];
    for(std::size_t i=0;i<3;i++)
      {
	ball_vel_new_in_contactee_coord_sys[i] =
	  ball_vel_in_contactee_coord_sys[i]*config.epsilon[i];
      }
    if (config.mirror_y)
      {
	ball_vel_new_in_contactee_coord_sys[1] =
	  -ball_vel_in_contactee_coord_sys[1]*config.epsilon[1]
	  + (1+config.epsilon[1])*contactee_vel_in_contactee_coord_sys[1];
      }
    else
      {
	ball_vel_new_in_contactee_coord_sys[2] =
	  -ball_vel_new_in_contactee_coord_sys[2];
      }
    

    double ball_pos_new_in_contactee_coord_sys[3];
    for(size_t i=0;i<3;i++)
      ball_pos_new_in_contactee_coord_sys[i] =
	ball_pos_in_contactee_coord_sys[i] +
	ball_vel_in_contactee_coord_sys[i]*time_until_impact +
	ball_vel_new_in_contactee_coord_sys[i]*delta_t_after_impact;

    double ball_pos_new[3];
    mju_rotVecQuat(ball_pos_new,
		   ball_pos_new_in_contactee_coord_sys,
		   quat_rot_neg);
    for(size_t i=0;i<3;i++)
      ball_pos_new[i] += pre_contact.contactee_position[i];
      
    double ball_vel_new[3];
    mju_rotVecQuat(ball_vel_new,
		   ball_vel_new_in_contactee_coord_sys,
		   quat_rot_neg);

    for(size_t i=0;i<3;i++)
      get_ball_position[i] = ball_pos_new[i];
    for(size_t i=0;i<3;i++)
      get_ball_velocity[i] = ball_vel_new[i]+ config.vel_plus[i];
  }
  
}
