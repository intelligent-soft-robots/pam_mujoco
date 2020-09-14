#include "pam_mujoco/recompute_state_after_contact.hpp"

namespace pam_mujoco
{

  RecomputeStateConfig::RecomputeStateConfig(){}
  
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


  template<int size>
  void print(std::string label, double a[size])
  {
    std::cout << label << " : ";
    for(int i=0;i<size;i++)
      {
	std::cout << a[i] << " , ";
      }
    std::cout << "\n";
  }

  template<int size>
  void print(std::string label, const std::array<double,size>& a)
  {
    std::cout << label << " : ";
    for(int i=0;i<size;i++)
      {
	std::cout << a[i] << " , ";
      }
    std::cout << "\n";
  }


  void in_relative_frame(const RecomputeStateConfig& config,
			 const internal::ContactStates& pre_contact,
			 std::array<double,3>& position,
			 std::array<double,3>& velocity)
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

    std::array<double,3> trans;
    for(size_t i=0;i<3;i++)
      {
	trans[i] = pre_contact.ball_position[i]-pre_contact.contactee_position[i];
      }
    
    mju_rotVecQuat(position.data(),
		   trans,
		   quat_rot);

    mju_rotVecQuat(velocity.data(),
		   pre_contact.contactee_velocity.data(),
		   quat_rot);
    
  }
  
  
  // generic version
  void _recompute_state_after_contact(const RecomputeStateConfig& config,
				     const internal::ContactStates& pre_contact,
				     const internal::ContactStates& current,
				     double get_ball_position[3],
				     double get_ball_velocity[3])
  {

    // position and velocity of the ball pre-contact,
    // in contactee (table or racket) frame
    std::array<double,3> pre_contact_relative_position;
    std::array<double,3> pre_contact_relative_velocity;
    in_relative_frame(config,pre_contact,
		      pre_contact_relative_position,
		      pre_contact_relative_velocity);

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
      {
	ball_pos_new_in_contactee_coord_sys[i] =
	  ball_pos_in_contactee_coord_sys[i] +
	  ball_vel_in_contactee_coord_sys[i]*time_until_impact +
	  ball_vel_new_in_contactee_coord_sys[i]*delta_t_after_impact;
      }
    
    double ball_pos_new[3];
    mju_rotVecQuat(ball_pos_new,
		   ball_pos_new_in_contactee_coord_sys,
		   quat_rot_neg);
    for(size_t i=0;i<3;i++)
      {
	ball_pos_new[i] += pre_contact.contactee_position[i];
      }
    
    double ball_vel_new[3];
    mju_rotVecQuat(ball_vel_new,
		   ball_vel_new_in_contactee_coord_sys,
		   quat_rot_neg);

    for(size_t i=0;i<3;i++)
      {
	get_ball_position[i] = ball_pos_new[i];
	get_ball_velocity[i] = ball_vel_new[i]+ config.vel_plus[i];
      }
  }


  // table version
  void recompute_state_after_contact(const RecomputeStateConfig& config,
				     const internal::ContactStates& pre_contact,
				     const internal::ContactStates& current,
				     double get_ball_position[3],
				     double get_ball_velocity[3])
  {
    std::array<double,3> pre_contact_relative;
    for(size_t i=0;i<3;i++)
      {
	pre_contact_relative[i] =
	  pre_contact.ball_position[i] - pre_contact.contactee_position[i];
      }
    pre_contact_relative[2]-=0.4; // !!

    std::array<double,3> post_contact_velocity;
    for(size_t i=0;i<3;i++)
      {
	post_contact_velocity[i] = pre_contact.ball_velocity[i]*config.epsilon[i];
      }
    post_contact_velocity[2] = - post_contact_velocity[2];

    
    
    double time_until_impact = -pre_contact_relative[2] / pre_contact.ball_velocity[2];
    double delta_t_step = current.time_stamp - pre_contact.time_stamp;
    double delta_t_after_impact = delta_t_step - time_until_impact;

    std::array<double,3> post_contact_relative;
    for (size_t i=0;i<3;i++)
      {
	post_contact_relative[i] =
	  pre_contact_relative[i] +
	  pre_contact.ball_velocity[i]*time_until_impact +
	  post_contact_velocity[i]*delta_t_after_impact;
      }

    std::array<double,3> post_contact;
    for (size_t i=0;i<3;i++)
      {
	post_contact[i] =
	  post_contact_relative[i] + pre_contact.contactee_position[i];
      }
    post_contact[2] += 0.04; // !!

    for(int i=0;i<3;i++)
      {
	get_ball_position[i] = post_contact[i];
	get_ball_velocity[i] = post_contact_velocity[i];
      }

  }

  
}
