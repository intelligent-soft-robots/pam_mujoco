

template <int QUEUE_SIZE, int NB_ITEMS>
MirrorFreeJoints<QUEUE_SIZE,NB_ITEMS>::MirrorFreeJoints(std::string segment_id,
					     std::array<std::string,NB_ITEMS> joint,
					     std::array<int,NB_ITEMS> index_qpos,
					     std::array<int,NB_ITEMS> index_qvel,
					     bool active_only)
    : segment_id_{segment_id},
      backend_{segment_id},
      joint_(joint),
      index_qpos_(index_qpos),
      index_qvel_(index_qvel),
      active_only_(active_only)
{
  contact_interrupt_.fill(false);
  interrupted_.fill(false);
}


template <int QUEUE_SIZE, int NB_ITEMS>
MirrorFreeJoints<QUEUE_SIZE,NB_ITEMS>::MirrorFreeJoints(std::string segment_id,
			      std::array<std::string,NB_ITEMS> joint,
			      std::array<int,NB_ITEMS> index_qpos,
			      std::array<int,NB_ITEMS> index_qvel,
			      std::array<std::string,NB_ITEMS> interrupt_segment_id,
			      bool active_only)
    : MirrorFreeJoints<QUEUE_SIZE,NB_ITEMS>::MirrorFreeJoints{
          segment_id, joint, index_qpos, index_qvel, active_only}
{
  for(int i=0;i<NB_ITEMS;i++)
    set_contact_interrupt(i,interrupt_segment_id[i]);
}

template <int QUEUE_SIZE, int NB_ITEMS>
void MirrorFreeJoints<QUEUE_SIZE,NB_ITEMS>::set_contact_interrupt(int index, std::string segment_id)
{
    contact_interrupt_[index] = true;
    segment_id_contact_[index] = segment_id;
}

template <int QUEUE_SIZE, int NB_ITEMS>
void MirrorFreeJoints<QUEUE_SIZE,NB_ITEMS>::apply(const mjModel* m, mjData* d)
{
    if (this->must_update(d))
    {

      for(int index=0;index<NB_ITEMS;index++)
	{
	  int root = index*6;
	  int index_qpos = index_qpos_[index];
	  int index_qvel = index_qvel_[index];
      
	  read_states_.values[root+0].value = d->qpos[index_qpos];
	  read_states_.values[root+2].value = d->qpos[index_qpos + 1];
	  read_states_.values[root+4].value = d->qpos[index_qpos + 2];
	  
	  read_states_.values[root+1].value = d->qvel[index_qvel];
	  read_states_.values[root+3].value = d->qvel[index_qvel + 1];
	  read_states_.values[root+5].value = d->qvel[index_qvel + 2];
	}

      set_states_ =
	backend_.pulse(o80::TimePoint(static_cast<long int>(d->time * 1e9)),
		       read_states_,
		       o80::VoidExtendedState());
    }

    bool active;
    if (active_only_)
	  {
	    active = backend_.is_active();
	  }
    else
      {
	active = true;
      }
    
    for(int index=0;index<NB_ITEMS;index++)
      {

	int root = index*6;
	int index_qpos = index_qpos_[index];
	int index_qvel = index_qvel_[index];
	
	// if the item has been set to have the trajectory interrupted
	// in case of contact (i.e. either customed model or mujoco engine
	// take hand), checking if such contact occured.
	// (note: see Contacts.hpp to see what serialize ContactInformation
	// instances into the shared memory)
	bool contact_disabled;
	if (contact_interrupt_[index])
	  {
	    context::ContactInformation ci;
	    shared_memory::deserialize(
				       segment_id_contact_[index], segment_id_contact_[index], ci);
	    contact_disabled = ci.disabled;
	    if (ci.contact_occured && !interrupted_[index])
	      {
		interrupted_[index] = true;
	      }
	    if (interrupted_[index] && !ci.contact_occured)
	      {
		// contact has been reset
		interrupted_[index] = false;
	      }
	  }

	// here mujoco's is overwritten by o80 desired state if:
	// 1a. we are not post contact (if interrupt_segment_id has been provided)
	// or
	// 1b. contact are disabled
	// and
	// 2. the backend is active (i.e. no o80 command is active)

	bool overwrite = ( ( (!interrupted_[index]) || contact_disabled ) && active );

	if (overwrite)
	  {
	    // x,y,z positions
	    d->qpos[index_qpos] = set_states_.get(root+0).value;
	    d->qpos[index_qpos + 1] = set_states_.get(root+2).value;
	    d->qpos[index_qpos + 2] = set_states_.get(root+4).value;
	    // x,y,z velocities
	    d->qvel[index_qvel] = set_states_.get(root+1).value;
	    d->qvel[index_qvel + 1] = set_states_.get(root+3).value;
	    d->qvel[index_qvel + 2] = set_states_.get(root+5).value;
	  }

      }
}

template <int QUEUE_SIZE, int NB_ITEMS>
void MirrorFreeJoints<QUEUE_SIZE,NB_ITEMS>::clear(std::string segment_id)
{
    o80::clear_shared_memory(segment_id);
}
