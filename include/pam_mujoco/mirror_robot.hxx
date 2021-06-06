

template <int QUEUE_SIZE, int NB_DOFS>
MirrorRobot<QUEUE_SIZE, NB_DOFS>::MirrorRobot(std::string segment_id,
                                              std::string robot_joint_base)
    : ControllerBase{},
      backend_{segment_id},
      robot_joint_base_(robot_joint_base),
      index_q_robot_(-1),
      index_qvel_robot_(-1)
{
}

template <int QUEUE_SIZE, int NB_DOFS>
bool MirrorRobot<QUEUE_SIZE, NB_DOFS>::same(const States& s1,
                                            const States& s2) const
{
    for (std::size_t dof = 0; dof < NB_DOFS; dof++)
    {
        o80::State2d t1 = s1.get(dof);
        o80::State2d t2 = s2.get(dof);
        if (t1.get<0>() != t2.get<0>())
        {
            return false;
        }
        if (t1.get<1>() != t2.get<1>())
        {
            return false;
        }
    }
    return true;
}

template <int QUEUE_SIZE, int NB_DOFS>
void MirrorRobot<QUEUE_SIZE, NB_DOFS>::apply(const mjModel* m, mjData* d)
{
    if (index_q_robot_ < 0)
    {
        index_q_robot_ = m->jnt_qposadr[mj_name2id(
            m, mjOBJ_JOINT, robot_joint_base_.c_str())];
        index_qvel_robot_ = m->jnt_dofadr[mj_name2id(
            m, mjOBJ_JOINT, robot_joint_base_.c_str())];
    }
    bool set_to_mujoco = false;
    if (this->must_update(d))
    {
        // must update means mujoco time stamp (d->time)
        // has been updated to a new iteration (i.e. increased of
        // 0.002s).
        o80::State2d joint_state;
        for (std::size_t dof = 0; dof < NB_DOFS; dof++)
        {
            joint_state.set<0>(d->qpos[index_q_robot_ + dof]);
            joint_state.set<1>(d->qvel[index_qvel_robot_ + dof]);
            read_states_.set(dof, joint_state);
        }
        set_states_ = backend_.pulse(
            this->get_time_stamp(), read_states_, o80::VoidExtendedState());
        if (!same(set_states_, previous_set_states_))
        {
            // there is a new mujoco iteration, and set_states_
            // has also been updated, so mujoco has to mirror this
            set_to_mujoco = true;
            previous_set_states_ = set_states_;
        }
    }
    else
    {
        // still working on previous iteration, so mujoco
        // needs update, possibly resetting the same values
        // again
        set_to_mujoco = true;
    }
    if (set_to_mujoco)
    {
        for (std::size_t dof = 0; dof < NB_DOFS; dof++)
        {
	  o80::State2d state = set_states_.get(dof);
	  if( (!std::isnan(state.get<0>())) && (!std::isnan(state.get<1>())) )
	    {
	      d->qpos[index_q_robot_ + dof] = state.get<0>();
	      d->qvel[index_qvel_robot_ + dof] = state.get<1>();
	    }
        }
    }
}

template <int QUEUE_SIZE, int NB_DOFS>
void MirrorRobot<QUEUE_SIZE, NB_DOFS>::clear(std::string segment_id)
{
    o80::clear_shared_memory(segment_id);
}
