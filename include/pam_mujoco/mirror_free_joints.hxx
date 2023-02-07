

template <int QUEUE_SIZE, int NB_ITEMS>
MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::MirrorFreeJoints(
    std::string mujoco_id,
    std::string segment_id,
    std::array<std::string, NB_ITEMS> joint,
    std::string geom_robot,
    bool active_only)
    : mujoco_id_{mujoco_id},
      segment_id_{segment_id},
      backend_{segment_id},
      joint_(joint),
      //index_qpos_(index_qpos),
      //index_qvel_(index_qvel),
      geom_robot_(geom_robot),
      index_robot_geom_{-1},
      active_only_(active_only)
{
    contact_interrupt_.fill(false);
    interrupted_.fill(false);
    index_qpos_.fill(-1);
    index_qvel_.fill(-1);
}

template <int QUEUE_SIZE, int NB_ITEMS>
MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::MirrorFreeJoints(
    std::string mujoco_id,
    std::string segment_id,
    std::array<std::string, NB_ITEMS> joint,
    std::string geom_robot,
    std::array<std::string, NB_ITEMS> interrupt_segment_id,
    bool active_only)
    : MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::MirrorFreeJoints{mujoco_id,
                                                               segment_id,
                                                               joint,
                                                               geom_robot,
                                                               active_only}
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
    if (index_robot_geom_ < 0)
    {
        index_robot_geom_ = mj_name2id(m, mjOBJ_GEOM, geom_robot_.c_str());
	for(int item=0; item<NB_ITEMS; item++)
	  {
	    index_qpos_[item] = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, joint_[item].c_str())];
	    index_qvel_[item] = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, joint_[item].c_str())];
	  }
    }

    // reading contacts from shared_memory (see contact_ball.hpp, to
    // see code of the controller writting this info in the shared memory)
    std::array<context::ContactInformation, NB_ITEMS> contact_information;
    std::array<bool, NB_ITEMS> contact_occured;
    // is any for the ball set to check for control interruption upong contact ?
    if (std::any_of(contact_interrupt_.begin(),
                    contact_interrupt_.end(),
                    [](bool v) { return v; }))
    {
        // looking at each ball
        for (int index = 0; index < NB_ITEMS; index++)
        {
            // true means here that the ball is set to have
            // its control interrupted post contact
            if (contact_interrupt_[index])
            {
                shared_memory::deserialize(segment_id_contact_[index],
                                           segment_id_contact_[index],
                                           contact_information[index]);
                // at least one contact occured between the ball and the
                // item (ball or robot, depending on configuration)
                contact_occured[index] =
                    contact_information[index].contact_occured;
            }
        }
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
                                                    d->qpos[index_qpos + 1],
                                                    d->qpos[index_qpos + 2]);
            read_states_.values[index].set_velocity(d->qvel[index_qvel],
                                                    d->qvel[index_qvel + 1],
                                                    d->qvel[index_qvel + 2]);
        }

        // reading the robot cartesian position
        std::array<double, 3> robot_position;
        for (int dim = 0; dim < 3; dim++)
        {
            robot_position[dim] = d->geom_xpos[index_robot_geom_ * 3 + dim];
        }

        // reading current episode
        long int episode;
        shared_memory::get<long int>(mujoco_id_, "episode", episode);

        // o80 information share
        set_states_ =
            backend_.pulse(o80::TimePoint(static_cast<long int>(d->time * 1e9)),
                           read_states_,
                           ExtraBallsExtendedState<NB_ITEMS>(
                               contact_occured, episode, robot_position));
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
        bool contact_disabled = false;
        // the ball is set to have control interrupted in case of contact
        if (contact_interrupt_[index])
        {
            // did user set contacts to be ignored ?
            contact_disabled = contact_information[index].disabled;
            // _interrupted is the state of the ball (_interrupted true
            // means the control was stopped due to previous contact)
            if (contact_occured[index] && !interrupted_[index])
            {
                // control was not interrupted so far, but a new contact
                // was detected, so interrupting control from now
                interrupted_[index] = true;
            }
            if (interrupted_[index] && !contact_occured[index])
            {
                // control was interrupted because of previous contact,
                // but contact information report there were no previous
                // contact: reinit occured
                interrupted_[index] = false;
            }
        }

        // here mujoco's is overwritten by o80 desired state if:
        // 1a. we are not post contact (if interrupt_segment_id has been
        // provided) or 1b. contact are disabled and
        // 2. the backend is active (continuous control, or active_only
        //                           control and there are new commands)

        bool overwrite =
            (((!interrupted_[index]) || contact_disabled) && active);

        if (overwrite)
        {
            for (int dim = 0; dim < 3; dim++)
            {
                d->qpos[index_qpos + dim] = set_states_.values[index].get(dim);
                d->qvel[index_qvel + dim] =
                    set_states_.values[index].get(3 + dim);
            }
        }
    }
}

template <int QUEUE_SIZE, int NB_ITEMS>
void MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::clear(std::string segment_id)
{
    o80::clear_shared_memory(segment_id);
}
