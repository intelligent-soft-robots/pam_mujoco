

template <int QUEUE_SIZE>
MirrorFreeJoint<QUEUE_SIZE>::MirrorFreeJoint(std::string segment_id,
                                             std::string joint,
                                             int index_qpos,
                                             int index_qvel,
                                             bool active_only)
    : segment_id_{segment_id},
      backend_{segment_id},
      joint_(joint),
      index_qpos_(index_qpos),
      index_qvel_(index_qvel),
      contact_interrupt_(false),
      interrupted_(false),
      active_only_(active_only)
{
}

template <int QUEUE_SIZE>
MirrorFreeJoint<QUEUE_SIZE>::MirrorFreeJoint(std::string segment_id,
                                             std::string joint,
                                             int index_qpos,
                                             int index_qvel,
                                             std::string interrupt_segment_id,
                                             bool active_only)
    : MirrorFreeJoint<QUEUE_SIZE>::MirrorFreeJoint{
          segment_id, joint, index_qpos, index_qvel, active_only}
{
    set_contact_interrupt(interrupt_segment_id);
}

template <int QUEUE_SIZE>
void MirrorFreeJoint<QUEUE_SIZE>::set_contact_interrupt(std::string segment_id)
{
    contact_interrupt_ = true;
    segment_id_contact_ = segment_id;
}

template <int QUEUE_SIZE>
void MirrorFreeJoint<QUEUE_SIZE>::apply(const mjModel* m, mjData* d)
{
    if (this->must_update(d))
    {
        read_states_.values[0].value = d->qpos[index_qpos_];
        read_states_.values[2].value = d->qpos[index_qpos_ + 1];
        read_states_.values[4].value = d->qpos[index_qpos_ + 2];

        read_states_.values[1].value = d->qvel[index_qvel_];
        read_states_.values[3].value = d->qvel[index_qvel_ + 1];
        read_states_.values[5].value = d->qvel[index_qvel_ + 2];

        set_states_ =
            backend_.pulse(o80::TimePoint(static_cast<long int>(d->time * 1e9)),
                           read_states_,
                           o80::VoidExtendedState());
    }

    // if the item has been set to have the trajectory interrupted
    // in case of contact (i.e. either customed model or mujoco engine
    // take hand), checking if such contact occured.
    // (note: see Contacts.hpp to see what serialize ContactInformation
    // instances into the shared memory)
    bool contact_disabled;
    if (contact_interrupt_)
    {
        context::ContactInformation ci;
        shared_memory::deserialize(
            segment_id_contact_, segment_id_contact_, ci);
	contact_disabled = ci.disabled;
        if (ci.contact_occured && !interrupted_)
        {
            interrupted_ = true;
        }
        if (interrupted_ && !ci.contact_occured)
        {
            // contact has been reset
            interrupted_ = false;
        }
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

    // here mujoco's is overwritten by o80 desired state if:
    // 1a. we are not post contact (if interrupt_segment_id has been provided)
    // or
    // 1b. contact are disabled
    // and
    // 2. the backend is active (i.e. no o80 command is active)

    bool overwrite = ( ( (!interrupted_) || contact_disabled ) && active );

    if (overwrite)
    {
        // x,y,z positions
        d->qpos[index_qpos_] = set_states_.get(0).value;
        d->qpos[index_qpos_ + 1] = set_states_.get(2).value;
        d->qpos[index_qpos_ + 2] = set_states_.get(4).value;
        // x,y,z velocities
        d->qvel[index_qvel_] = set_states_.get(1).value;
        d->qvel[index_qvel_ + 1] = set_states_.get(3).value;
        d->qvel[index_qvel_ + 2] = set_states_.get(5).value;
    }
}

template <int QUEUE_SIZE>
void MirrorFreeJoint<QUEUE_SIZE>::clear(std::string segment_id)
{
    o80::clear_shared_memory(segment_id);
}
