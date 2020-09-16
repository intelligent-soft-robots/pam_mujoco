#include "pam_mujoco/contact_ball.hpp"

namespace pam_mujoco
{

  namespace internal
  {

    /**
     * extract information from d to update the instance
     * of get_states. position of the ball, velocity of the ball,
     * position of the contactee and orientation of the contactee
     * are directly read from d.
     * the velocity of the contactee is computed via finite difference
     * using the position of the contactee as provided by get_states.
     * (i.e. get_states is also expected to encapsulte the previous
     * state).
     */
    void save_state(const mjData* d,
		    int index_qpos,
		    int index_qvel,
		    int index_geom,
		    int index_geom_contactee,
		    internal::ContactStates& get_states,
		    bool verbose)
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
	  if (delta_time!=0)
	    {
	      for(size_t i=0;i<3;i++)
		{
		  get_states.contactee_velocity[i] =
		    (d->geom_xpos[index_geom_contactee*3+i]-get_states.contactee_position[i]) /
		    delta_time;
		}
	    }
	}
      // rest is just copied from d to get_states
      for(size_t i=0;i<3;i++)
	{
	  get_states.ball_position[i] = d->qpos[index_qpos+i];
	  get_states.ball_velocity[i] = d->qvel[index_qvel+i];
	  get_states.contactee_position[i] = d->geom_xpos[index_geom_contactee*3+i];
	}
      for(size_t i=0;i<9;i++)
	{
	  get_states.contactee_orientation[i] = d->geom_xmat[index_geom_contactee*9+i];
	}
      get_states.time_stamp = d->time;
    }

  }  

  ContactBall::ContactBall(std::string segment_id_contact_info,
			   std::string segment_id_reset,
			   int index_qpos,
			   int index_qvel,
			   std::string geom,
			   std::string geom_contactee,
			   ContactItems contact_item)
    : segment_id_contact_info_(segment_id_contact_info),
      segment_id_reset_(segment_id_reset),
      config_(get_recompute_state_config(contact_item)),
      index_qpos_(index_qpos),
      index_qvel_(index_qvel),
      geom_(geom),
      geom_contactee_(geom_contactee),
      index_geom_(-1),
      index_geom_contactee_(-1)
  {
    shared_memory::clear_shared_memory(segment_id_contact_info_);
    shared_memory::clear_shared_memory(segment_id_reset_);
    shared_memory::set<bool>(segment_id_reset_,
			     segment_id_reset_,
			     false);
    shared_memory::serialize(segment_id_contact_info_,
			     segment_id_contact_info_,
			     contact_information_);
  }

  void ContactBall::apply(const mjModel* m,
			      mjData* d)
  {

    // setup the indexes (e.g. index_qpos, ...)
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

    internal::ContactAction contact_action = contact_logic_.apply(m,d,
								  index_geom_,
								  index_geom_contactee_);
    
    if (contact_action.muted)
      {
	return;
      }

    if (contact_action.in_contact)
      {
	internal::ContactStates current;
	internal::save_state(d,
			     index_qpos_,
			     index_qvel_,
			     index_geom_,
			     index_geom_contactee_,
			     current,false);
	contact_information_.register_contact(current.ball_position,
					      d->time);
	recompute_state_after_contact(config_,
				      previous_,
				      current,
				      &(d->qpos[index_qpos_]),
				      &(d->qvel[index_qvel_]));
      }
    else
      {
	internal::save_state(d,
			     index_qpos_,
			     index_qvel_,
			     index_geom_,
			     index_geom_contactee_,
			     previous_,true);
	double d_ball_contactee = mju_dist3(previous_.ball_position.data(),
					    previous_.contactee_position.data());
	contact_information_.register_distance(d_ball_contactee);
      }

    shared_memory::serialize(segment_id_contact_info_,
			     segment_id_contact_info_,
			     contact_information_);

  }
  

  void ContactBall::init(const mjModel* m)
  {
    if(index_geom_>=0)
      {
	// init does something only at first call
	return;
      }
    index_geom_ = mj_name2id(m, mjOBJ_GEOM,
			     geom_.c_str());
    index_geom_contactee_ = mj_name2id(m, mjOBJ_GEOM,
				       geom_contactee_.c_str());
  }
  
  
  void ContactBall::reset()
  {
    contact_information_ = ContactInformation{};
    previous_ = internal::ContactStates{};
  }

}
