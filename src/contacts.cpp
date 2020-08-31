#include "pam_mujoco/contacts.hpp"

namespace pam_mujoco
{

  ContactInformation::ContactInformation()
    : position{0,0,0},
      contact_occured(false),
      minimal_distance(-1),
      time_stamp(-1)
  {}

  void ContactInformation::register_distance(double d)
  {
    if(minimal_distance<0)
      {
	minimal_distance=d;
	return;
      }
    minimal_distance = std::min(minimal_distance,
				d);
  }

  void ContactInformation::register_contact(std::array<double,3> _position,
					    double _time_stamp)
  {
    contact_occured=true;
    minimal_distance = 0;
    position = _position;
    time_stamp = _time_stamp;
  }


  
  ContactStates::ContactStates()
    : time_stamp(-1){}


  
  void recompute_state_after_contact(const RecomputeStateConfig& config,
				     const ContactStates& pre_contact,
				     const ContactStates& current,
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

    double time_until_impact = -ball_pos_in_contactee_coord_sys[1] /
      (ball_vel_in_contactee_coord_sys[1] - contactee_vel_in_contactee_coord_sys[1]);
    double delta_t_step = current.time_stamp - pre_contact.time_stamp;
    double delta_t_after_impact = delta_t_step - time_until_impact;

    double ball_vel_new_in_contactee_coord_sys[3];
    ball_vel_new_in_contactee_coord_sys[0] =
      ball_vel_in_contactee_coord_sys[0]*config.epsilon_r;
    ball_vel_new_in_contactee_coord_sys[1] =
      -ball_vel_in_contactee_coord_sys[1]*config.epsilon_r
      + (1+config.epsilon_r)*contactee_vel_in_contactee_coord_sys[1];
    ball_vel_new_in_contactee_coord_sys[2] =
      ball_vel_in_contactee_coord_sys[2]*config.epsilon_r;

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
    get_ball_velocity[0] = ball_vel_new[0];
    get_ball_velocity[1] = ball_vel_new[1] + config.y_vel_plus;
    get_ball_velocity[2] = ball_vel_new[2] + config.z_vel_plus;
  }


  void save_state(const mjData* d,
		  int index_q_ball,
		  int index_qvel_ball,
		  int index_geom_ball,
		  int index_geom_contactee,
		  ContactStates& get_states)
  {
    // velocity of contactee computed with finite differences
    if(get_states.time_stamp<0)
      {
	for(size_t i=0;i<3;i++)
	  get_states.contactee_velocity[i]=0;
      }
    else
      {
	double delta_time = d->time - get_states.time_stamp;
	for(size_t i=0;i<3;i++)
	  get_states.contactee_velocity[i] =
	    (d->geom_xpos[index_geom_contactee*3+i]-get_states.contactee_position[i]) /
	    delta_time;
      }
    // rest is just copied from d to get_states
    for(size_t i=0;i<3;i++)
      {
	get_states.ball_position[i] = d->qpos[index_q_ball+i];
	get_states.ball_velocity[i] = d->qvel[index_qvel_ball+i];
	get_states.contactee_position[i] = d->geom_xpos[index_geom_contactee*3+i];
      }
    for(size_t i=0;i<9;i++)
      {
	get_states.contactee_orientation[i] = d->geom_xmat[index_geom_contactee*9+i];
      }
    get_states.time_stamp = d->time;
  }

  

  RecomputeStateConfig::RecomputeStateConfig(bool robot1)
    : epsilon_r(0.78),
      epsilon_t_x(0.73),
      epsilon_t_z(0.92),
      y_vel_plus(0),
      z_vel_plus(0)
  {
    if(robot1)
      {
	rot_matrix_contactee_zero_pos = {0.000, -1.000, -0.000,
				      -1.000, 0.000, -0.000,
				      0.000, 0.000, -1.000};
      }
    else {
      rot_matrix_contactee_zero_pos = {-0.000, 1.000, 0.000,
				    1.000, -0.000, 0.000,
				    0.000, 0.000, -1.000};
    }
  }

  RecomputeStateConfig::RecomputeStateConfig()
    : epsilon_r(0.78),
      epsilon_t_x(0.73),
      epsilon_t_z(0.92),
      y_vel_plus(0),
      z_vel_plus(0)
  {
    rot_matrix_contactee_zero_pos = {0.000, -1.000, -0.000,
				     -1.000, 0.000, -0.000,
				     0.000, 0.000, -1.000};
  }

  ContactBall::ContactBall(std::string segment_id_contact_info,
			   std::string segment_id_reset,
			   RecomputeStateConfig config,
			   std::string ball_obj_joint,
			   std::string ball_geom,
			   std::string contactee_geom)
    : segment_id_contact_info_(segment_id_contact_info),
      segment_id_reset_(segment_id_reset),
      config_(config),
      index_q_ball_(-1),
      index_qvel_ball_(-1),
      index_geom_ball_(-1),
      index_geom_contactee_(-1),
      ball_obj_joint_(ball_obj_joint),
      ball_geom_(ball_geom),
      contactee_geom_(contactee_geom),
      muted_(false)
  {
    shared_memory::set<bool>(segment_id_reset_,
			     segment_id_reset_,
			     false);
  }

  void ContactBall::apply(const mjModel* m,
			  mjData* d)
  {
    // setup the indexes (e.g. index_q_ball, ...)
    // based on the model and the configuration string
    init(m);
    // check if receiving from outside a request for reset
    // (e.g. a new episode starts)
    bool must_reset;
    shared_memory::get<bool>(segment_id_reset_,
			     segment_id_reset_,
			     must_reset);
    // reset : reinitialize ContactStates previous_
    //         and contact_information_
    if(must_reset)
      {
	reset();
	shared_memory::set<bool>(segment_id_reset_,
				 segment_id_reset_,
				 false);
      }
    // detect contacts, and if contact is detected,
    // update the position and velocity of the ball
    // encapsulated in d.
    // also update contact_information_
    manage_contact(m,d);
    // sharing contact_information_ with the
    // rest of the world
    shared_memory::serialize(segment_id_contact_info_,
			     segment_id_contact_info_,
			     contact_information_);
  }
  

  void ContactBall::init(const mjModel* m)
  {
    if(index_q_ball_>=0)
      // init does something only at first call
      return;
    index_q_ball_ = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT,
					      ball_obj_joint_.c_str())];
    index_qvel_ball_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT,
						ball_obj_joint_.c_str() )];
    index_geom_ball_ = mj_name2id(m, mjOBJ_GEOM,
				  ball_geom_.c_str());
    index_geom_contactee_ = mj_name2id(m, mjOBJ_GEOM,
				       contactee_geom_.c_str());
  }
  
  bool ContactBall::is_in_contact(const mjModel* m, mjData* d)
  {
    for(int i=0;i<d->ncon;i++)
      {
	if(d->contact[i].geom1 == index_geom_ball_
	   && d->contact[i].geom2 == index_geom_contactee_)
	  return true;
	if(d->contact[i].geom2 == index_geom_ball_
	   && d->contact[i].geom1 == index_geom_contactee_)
	  return true;
      }
    return false;
  }
  
  void ContactBall::manage_contact(const mjModel* m, mjData* d)
  {
    // check if mujoco reports contact ball/contactee
    bool contact = is_in_contact(m,d);
    // if muted (i.e. contact detected, but not the first iteration
    // it is detected), then we ignore all (i.e. the instance is "muted")
    if(is_muted(contact))
      {
	return;
      }
    // no contact, saving the state (in previous_), and 
    // updating contact_information_ with the distance ball/observee
    // (contact_information_ tracks for the smaller distance ever observed)
    if(!contact)
      {
	save_state(d,
		   index_q_ball_,
		   index_qvel_ball_,
		   index_geom_ball_,
		   index_geom_contactee_,
		   previous_);
	double d_ball_contactee = mju_dist3(previous_.ball_position.data(),
					 previous_.contactee_position.data());
	contact_information_.register_distance(d_ball_contactee);
	return;
      }
    // first detection of contact (otherwise muted_ would have been true)
    // correcting the ball trajectory if necessary
    // updating contact_information_ with info regarding detected contact
    ContactStates current;
    save_state(d,
	       index_q_ball_,
	       index_qvel_ball_,
	       index_geom_ball_,
	       index_geom_contactee_,
	       current);
    contact_information_.register_contact(current.ball_position,
					  d->time);
    // correcting d->qpos and d->qvel of the ball based
    // on customized contact model
    recompute_state_after_contact(config_,
				  previous_,
				  current,
				  &(d->qpos[index_q_ball_]),
				  &(d->qvel[index_qvel_ball_]));
  }
  
  bool ContactBall::is_muted(bool contact_detected)
  {
    if(!contact_detected)
      {
	muted_=false;
	return false;
      }
    if(!muted_)
      {
	muted_=true;
	muted_count_=0;
	return true;
      }
    muted_count_++;
    if(muted_count_>=20)
      {
	muted_=false;
	return false;
      }
    return true;
  }

  void ContactBall::reset()
  {
    contact_information_ = ContactInformation{};
    previous_ = ContactStates{};
  }

}
