
ContactInterrupt::ContactInterrupt(std::string segment_id)
    : segment_id_{segment_id},
      interrupted_{false}
{}

std::tuple<bool,bool> ContactInterrupt::interrupted()
{
    shared_memory::deserialize(segment_id_, segment_id_, ci_);
    bool contact_disabled = ci_.disabled;
    if (ci_.contact_occured && !interrupted_)
        {
            interrupted_ = true;
        }
    if (interrupted_ && !ci_.contact_occured)
        {
            // contact has been reset
            interrupted_ = false;
        }
    return std::make_tuple(contact_disabled,interrupted_);
}



template <int QUEUE_SIZE>
MirrorFreeJoint<QUEUE_SIZE>::MirrorFreeJoint(std::string segment_id,
                                             std::string joint,
                                             bool active_only)
    : backend_{segment_id},
      segment_id_{segment_id},
      joint_(joint),
      index_qpos_(-1),
      index_qvel_(-1),
      contact_interrupt_(false),
      active_only_(active_only)
{
}

template <int QUEUE_SIZE>
MirrorFreeJoint<QUEUE_SIZE>::MirrorFreeJoint(std::string segment_id,
                                             std::string joint,
                                             const std::vector<std::string>& interrupt_segment_ids,
                                             bool active_only)
    : MirrorFreeJoint<QUEUE_SIZE>::MirrorFreeJoint{
          segment_id, joint, active_only}
{
    set_contact_interrupt(interrupt_segment_ids);
}

template <int QUEUE_SIZE>
void MirrorFreeJoint<QUEUE_SIZE>::set_contact_interrupt(const std::vector<std::string>& segment_ids)
{
    contact_interrupt_ = true;
    for(const std::string& segment_id: segment_ids)
        {
            contact_interrupts_.push_back(ContactInterrupt(segment_id));
        }
}

// only overwrite if new ball state
template <int QUEUE_SIZE>
bool MirrorFreeJoint<QUEUE_SIZE>::same(const States& s1, const States& s2) const
{
    for (std::size_t d = 0; d < 6; d++)
    {
        o80::State1d t1 = s1.get(d);
        o80::State1d t2 = s2.get(d);
        if (std::abs(t1.value - t2.value) > 1e-5)
        {
            return false;
        }
    }
    return true;
}



template <int QUEUE_SIZE>
void MirrorFreeJoint<QUEUE_SIZE>::apply(const mjModel* m, mjData* d)
{
    if (index_qpos_ < 0)
    {
        index_qpos_ =
            m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, joint_.c_str())];
        index_qvel_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, joint_.c_str())];
    }

    (void)(m);
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
    bool contact_disabled = false;
    bool interrupted = false;
    if (contact_interrupt_)
    {
        for(ContactInterrupt& ci: contact_interrupts_)
            {
                std::tuple<bool,bool> t = ci.interrupted();
                bool _contact_disabled = std::get<0>(t);
                bool _interrupted = std::get<1>(t);
                if(_contact_disabled)
                    contact_disabled = true;
                if(_interrupted)
                    interrupted = true;
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

    bool overwrite = (((!interrupted) || contact_disabled) && active);

    // only overwrite if new ball state
    // (to let mujoco play its course between ball trajectory points)
    // commented: this is redundant with the condition "if(active_only_)"
    //            right above.

    /*
    if (overwrite && same(set_states_, previous_set_states_))
    {
        if (must_update_counter_ > 0)
        {
            overwrite = true;
            must_update_counter_--;
        }
        else
        {
            overwrite = false;
        }
    }
    else{
        must_update_counter_ = 4;
    }
    previous_set_states_ = set_states_;
    */

    if (overwrite)
    {
        bool anynan = false;
        for (int i = 0; i < 6; i++)
        {
            if (std::isnan(set_states_.get(i).value))
            {
                anynan = true;
                break;
            }
        }
        if (!anynan)
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
}

template <int QUEUE_SIZE>
void MirrorFreeJoint<QUEUE_SIZE>::clear(std::string segment_id)
{
    o80::clear_shared_memory(segment_id);
}
