#include "pam_mujoco/contact_ball.hpp"

namespace pam_mujoco
{

ContactBall::ContactBall(std::string segment_id,
                         int index_qpos,
                         int index_qvel,
                         std::string geom,
                         std::string geom_contactee,
                         ContactItems contact_item)
  : segment_id_{segment_id},
    config_{internal::get_recompute_state_config(contact_item)},
    index_qpos_{index_qpos},
    index_qvel_{index_qvel},
    geom_{geom},
    geom_contactee_{geom_contactee},
    index_geom_{-1},
    index_geom_contactee_{-1},
    in_contact_{false},
    nb_of_iterations_since_last_contact_{-1}
{
    shared_memory::clear_shared_memory(segment_id_);
    shared_memory::set<bool>(segment_id_, "reset", false);
    shared_memory::set<bool>(segment_id_, "activated", true);
    shared_memory::serialize(segment_id_, segment_id_, contact_information_);
}

  template<int DIM, class P, class V>
  void printxd(P p, V v)
  {
    for(int dim=0;dim<DIM;dim++)
      {
	std::cout << p[dim] << " , ";
      }
    std::cout << " | ";
    for(int dim=0;dim<DIM;dim++)
      {
	std::cout << v[dim] << " , ";
      }
  }

  
  bool ContactBall::update(const mjModel* m, mjData* d)
  {

    // check if receiving from outside a request for reset
    // (e.g. a new episode starts)
    bool must_reset;
    shared_memory::get<bool>(segment_id_, "reset", must_reset);
    if (must_reset)
      {
	reset();
	shared_memory::set<bool>(segment_id_, "reset", false);
      }

    // checking if the contacts are activated
    bool activated;
    shared_memory::get<bool>(segment_id_, "activated", activated);
    if (!activated)
      {
	contact_information_.disabled = true;
	return false;
      }
    else
      {
	contact_information_.disabled = false;
      }
    
    // if there have been a recent contact,
    // contacts are ignored for a while (to avoid
    // the "same" contact to be applied twice)
    if(nb_of_iterations_since_last_contact_>=0)
      {
	if(nb_of_iterations_since_last_contact_<NB_ITERATIONS_CONTACT_MUTED)
	  {
	    nb_of_iterations_since_last_contact_++;
	    return false;
	  }
	else
	  {
	    // reactivating contact detection
	    nb_of_iterations_since_last_contact_=-1;
	  }
      }
    // contact are not being ignored
    // does mujoco reports a contact ?
    bool in_contact = internal::is_in_contact(m,d,
					      index_geom_, index_geom_contactee_);
    // no, so exiting
    if(! in_contact)
      {
	// before exiting, saving the state of the ball.
	// it will be used if at the next iteration there
	// is a contact (to compute velocity)
	internal::save_state(d,
			     index_qpos_,
			     index_qvel_,
			     index_geom_contactee_,
			     previous_);
	// saving also the distance between the ball
	// and the contactee.
	// computing the distance between the 2 objects
        double d_ball_contactee =
	  mju_dist3(previous_.ball_position.data(),
		    previous_.contactee_position.data());
        // updating contact_information with this distance. register_distance
        // will check wether or not this is the smallest distance ever observed,
        // and if so, save it as minimal distance
        contact_information_.register_distance(d_ball_contactee);
	return false;
      }
    
    // there is a contact: applying the custom contact model
    // first, converting data encapsulated by "d"
    // into an instance of ContactStates 
    internal::ContactStates current;
    internal::save_state(d,
			 index_qpos_,
			 index_qvel_,
			 index_geom_contactee_,
			 current);

    std::cout << "contact:" << std::endl;
    std::cout << "\tprevious:\n";
    std::cout << "\t\ttable orientation:" << std::endl;
    printxd<9>(previous_.contactee_orientation,previous_.contactee_orientation);
    std::cout << "\n\t\ttable:" << std::endl;
    printxd<3>(previous_.contactee_position,previous_.contactee_velocity);
    std::cout << "\n\t\tball:" << std::endl;
    printxd<3>(previous_.ball_position,previous_.ball_velocity);
    std::cout << "\n\tcurrent:\n";
    std::cout << "\t\ttable orientation:" << std::endl;
    printxd<9>(current.contactee_orientation,current.contactee_orientation);
    std::cout << "\n\t\ttable:" << std::endl;
    printxd<3>(current.contactee_position,current.contactee_velocity);
    std::cout << "\n\t\tball:" << std::endl;
    printxd<3>(current.ball_position,current.ball_velocity);
    
    // the command below updates overwite_ball_position_
    // and overwrite_ball_velocity_ with the values
    // commanded by the contact_model
    recompute_state_after_contact(config_,
				  previous_,
				  current,
				  overwrite_ball_position_,
				  overwrite_ball_velocity_);

    std::cout << "\n\toutput:\n";
    printxd<3>(overwrite_ball_position_,overwrite_ball_velocity_);
    std::cout << std::endl;
    
    // to "shut down" the few next contact detection
    // (which may come from the same contact)
    nb_of_iterations_since_last_contact_=0;
    // informing the outside world about the contact
    // (via shared memory)
    contact_information_.register_contact(current.ball_position, d->time);
    shared_memory::serialize(segment_id_, segment_id_, contact_information_);
    return true;
  }


void ContactBall::apply(const mjModel* m, mjData* d)
{

    // checking if it is a new mujoco iteration
    if (this->must_update(d))
    {
      
      // setup the indexes (e.g. index_qpos, ...)
      // based on the model and the configuration string
      // (has effect only at first call)
      init(m);

      // updating the variables:
      // - contact_information_ // information about contacts, for the external
      //                           world (written in shared memory)
      // - overwrite_ball_position_ // position as computed by custom contact model
      // - overwrite_ball_velocity_ // velocity as computed by custom contact model
      in_contact_ = update(m,d);

      // sharing contact information with the world
      shared_memory::serialize(segment_id_, segment_id_, contact_information_);
      
    }

    // no contact, so nothing to do
    if(! in_contact_)
      {
	return;
      }

    // we reach here only if in contact,
    // so overwritting mujoco data
    // with custom contact model data
    for(int dim=0;dim<3;dim++)
      {
	(&(d->qpos[index_qpos_]))[dim]=overwrite_ball_position_[dim];
	(&(d->qvel[index_qvel_]))[dim]=overwrite_ball_velocity_[dim];
      }

}

void ContactBall::init(const mjModel* m)
{
    if (index_geom_ >= 0)
    {
        // init does something only at first call
        return;
    }
    index_geom_ = mj_name2id(m, mjOBJ_GEOM, geom_.c_str());
    index_geom_contactee_ = mj_name2id(m, mjOBJ_GEOM, geom_contactee_.c_str());
}

void ContactBall::reset()
{
    contact_information_ = context::ContactInformation{};
    previous_ = internal::ContactStates{};
    nb_of_iterations_since_last_contact_ = -1;
}

void reset_contact(const std::string& segment_id)
{
  shared_memory::set<bool>(segment_id, "reset", true);
}

void activate_contact(const std::string& segment_id)
{
    shared_memory::set<bool>(segment_id, "activated", true);
}

void deactivate_contact(const std::string& segment_id)
{
    shared_memory::set<bool>(segment_id, "activated", false);
}

}  // namespace pam_mujoco
