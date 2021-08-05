

template <int QUEUE_SIZE, int NB_ITEMS>
MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::MirrorFreeJoints(
							 std::string mujoco_id,
							 std::string segment_id,
    std::array<std::string, NB_ITEMS> joint,
    std::array<int, NB_ITEMS> index_qpos,
    std::array<int, NB_ITEMS> index_qvel,
    std::string robot_joint_base,
    bool active_only)
  : mujoco_id_{mujoco_id},
    segment_id_{segment_id},
      backend_{segment_id},
      joint_(joint),
      index_qpos_(index_qpos),
      index_qvel_(index_qvel),
      robot_joint_base_(robot_joint_base_),
      index_robot_geom_{-1},
      active_only_(active_only)
{
    contact_interrupt_.fill(false);
    interrupted_.fill(false);
}

template <int QUEUE_SIZE, int NB_ITEMS>
MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::MirrorFreeJoints(
							 std::string mujoco_id,
    std::string segment_id,
    std::array<std::string, NB_ITEMS> joint,
    std::array<int, NB_ITEMS> index_qpos,
    std::array<int, NB_ITEMS> index_qvel,
    std::string robot_joint_base,
    std::array<std::string, NB_ITEMS> interrupt_segment_id,
    bool active_only)
  : MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::MirrorFreeJoints{mujoco_id,
  segment_id, joint, index_qpos, index_qvel, robot_joint_base,active_only } 
{
    for (int i = 0; i < NB_ITEMS; i++)
        set_contact_interrupt(i, interrupt_segment_id[i]);
}

template <int QUEUE_SIZE, int NB_ITEMS>
void MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::set_contact_interrupt(
    int index, std::string segment_id)
{
    contact_interrupt_[index] = true;
    segment_id_contact_[index] = segment_id;
}

template <int QUEUE_SIZE, int NB_ITEMS>
void MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::apply(const mjModel* m, mjData* d)
{
  // init, first iteration only
  if(index_robot_geom_<0)
    {
      index_robot_geom_ = mj_name2id(m, mjOBJ_GEOM, robot_joint_base_.c_str());
    }

  // reading contacts
  std::array<context::ContactInformation,NB_ITEMS> contact_information;
  std::array<bool,NB_ITEMS> contact_occured;
  for(int index=0;index<NB_ITEMS;index++)
    {
      shared_memory::deserialize(segment_id_contact_[index],
				 segment_id_contact_[index],
				 contact_information[index]);
      contact_occured[index]=contact_information[index].contact_occured;
    }

  // things to do only if new time step
  // (o80 call)
    if (this->must_update(d))
    {

        // reading the position and velocity of the items
        for (int index = 0; index < NB_ITEMS; index++)
        {
            int index_qpos = index_qpos_[index];
            int index_qvel = index_qvel_[index];

	    read_states_.values[index].set_position(d->qpos[index_qpos],
						    d->qpos[index_qpos+1],
						    d->qpos[index_qpos+2]);
	    read_states_.values[index].set_velocity(d->qvel[index_qvel],
						    d->qvel[index_qvel+1],
						    d->qvel[index_qvel+2]);
	    
        }

	// reading the robot cartesian position
	std::array<double,3> robot_position;
	for(int dim=0;dim<3;dim++)
	  {
	    robot_position[dim]=d->geom_xpos[index_robot_geom_*3+dim];
	  }

	// reading current episode
	long int episode;
	shared_memory::get<long int>(mujoco_id_,"episode",episode);

	// o80 information share
        set_states_ =
            backend_.pulse(o80::TimePoint(static_cast<long int>(d->time * 1e9)),
                           read_states_,
                           ExtraBallsExtendedState<NB_ITEMS>(contact_occured,
							     episode,
							     robot_position));
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

    for (int index = 0; index < NB_ITEMS; index++)
    {
        int index_qpos = index_qpos_[index];
        int index_qvel = index_qvel_[index];

        // if the item has been set to have the trajectory interrupted
        // in case of contact (i.e. either customed model or mujoco engine
        // take hand), checking if such contact occured.
        // (note: see Contacts.hpp to check what serializes ContactInformation
        // instances into the shared memory)
        bool contact_disabled;
        if (contact_interrupt_[index])
        {
            contact_disabled = contact_information[index].disabled;
            if (contact_occured[index] && !interrupted_[index])
            {
                interrupted_[index] = true;
            }
            if (interrupted_[index] && !contact_occured[index])
            {
                // contact has been reset
                interrupted_[index] = false;
            }
        }

        // here mujoco's is overwritten by o80 desired state if:
        // 1a. we are not post contact (if interrupt_segment_id has been
        // provided) or 1b. contact are disabled and
        // 2. the backend is active (i.e. no o80 command is active)

        bool overwrite =
            (((!interrupted_[index]) || contact_disabled) && active);

        if (overwrite)
        {
	  for (int dim=0;dim<3;dim++)
	    {
	      d->qpos[index_qpos+dim] = set_states_.values[index].get(dim);
	      d->qvel[index_qvel+dim] = set_states_.values[index].get(3+dim);
	    }
        }
    }
}


template <int QUEUE_SIZE, int NB_ITEMS>
void MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::clear(std::string segment_id)
{
    o80::clear_shared_memory(segment_id);
}
