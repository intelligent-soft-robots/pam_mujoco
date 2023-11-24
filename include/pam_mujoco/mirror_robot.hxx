
template <int QUEUE_SIZE, int NB_DOFS>
MirrorRobot<QUEUE_SIZE, NB_DOFS>::MirrorRobot(std::string segment_id,
                                              std::string robot_joint_base,
                                              std::string robot_racket)
    : ControllerBase{},
      backend_{segment_id},
      robot_joint_base_(robot_joint_base),
      robot_racket_(robot_racket),
      index_geom_(-1),
      index_q_robot_(-1),
      index_qvel_robot_(-1)
{
    racket_state_ = internal::ContactStates{};
}

template <int QUEUE_SIZE, int NB_DOFS>
bool MirrorRobot<QUEUE_SIZE, NB_DOFS>::same(const States& s1,
                                            const States& s2) const
{
    for (std::size_t dof = 0; dof < NB_DOFS; dof++)
    {
        o80::State2d t1 = s1.get(dof);
        o80::State2d t2 = s2.get(dof);
        // only overwrite if new robot state (compare numbers, instead of t1.get<0>() != t2.get<0>() as previously)
        if (std::abs(t1.get<0>() - t2.get<0>()) > 1e-5)
        {
            return false;
        }
        if (std::abs(t1.get<0>() - t2.get<0>()) > 1e-5)
        {
            return false;
        }
    }
    return true;
}

template <int QUEUE_SIZE, int NB_DOFS>
void MirrorRobot<QUEUE_SIZE, NB_DOFS>::update_robot_fk(const mjData* d)
{
    internal::save_state(
            d, index_q_robot_, 0, 0, index_geom_, racket_state_);


    for (int dim = 0; dim < 3; dim++)
    {
        // robot_fk_.set_position(dim, d->geom_xpos[index_geom_ * 3 + dim]);
        robot_fk_.set_position(dim, racket_state_.contactee_position[dim]);
    }
    for (int dim = 0; dim < 3; dim++)
    {
        robot_fk_.set_velocity(dim, racket_state_.contactee_velocity[dim]);
    }
    // printf("contactee and joint velocity: %f %f %f %f %f %f %f %f %f %f\n", d->time, racket_state_.contactee_velocity[0], racket_state_.contactee_velocity[1], racket_state_.contactee_velocity[2], d->qvel[index_qvel_robot_], d->qvel[index_qvel_robot_+1], d->qvel[index_qvel_robot_+2], d->qpos[index_q_robot_], d->qpos[index_q_robot_+1], d->qpos[index_q_robot_+2]);
    for (int dim = 0; dim < 9; dim++)
    {
        //robot_fk_.set_orientation(dim, d->geom_xmat[index_geom_ * 9 + dim]);
        robot_fk_.set_orientation(dim, racket_state_.contactee_orientation[dim]);
    }
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
        index_geom_ = mj_name2id(m, mjOBJ_GEOM, robot_racket_.c_str());
    }

    bool set_to_mujoco = false;

    if (this->must_update(d))
    {
        // must_update means mujoco time stamp (d->time)
        // has been updated to a new iteration (i.e. increased of
        // 0.002s).

        // reading robot state
        o80::State2d joint_state;
        for (std::size_t dof = 0; dof < NB_DOFS; dof++)
        {
            joint_state.set<0>(d->qpos[index_q_robot_ + dof]);
            joint_state.set<1>(d->qvel[index_qvel_robot_ + dof]);
            read_states_.set(dof, joint_state);
        }

        // getting robot fk as extended state
        update_robot_fk(d);

        // writting current state to o80, getting the desired state back
        // (set_states_: state that o80 should enforce on mujoco)
        set_states_ =
            backend_.pulse(this->get_time_stamp(), read_states_, robot_fk_);

        // updating mujoco if there is a change of desired state
        if (!same(set_states_, previous_set_states_))
        {
            // there is a new mujoco iteration, and set_states_
            // has also been updated, so mujoco has to mirror this
            must_update_counter_ = 4;   // update for 4 steps to make sure update is not overwritten by mujoco
            previous_set_states_ = set_states_;
        }

        // update mujoco if all velocities are zero, hacky and doesn't always work, necessary because overwrite happens only if new robot state
        if (must_update_counter_ < 1)
        {
            bool all_velocities_zero = true;
            for (std::size_t dof = 0; dof < NB_DOFS; dof++)
            {
                o80::State2d state = set_states_.get(dof);
                if (abs(state.get<1>()) > 0.01)
                {
                    all_velocities_zero = false;
                }
            }
            if (all_velocities_zero)
            {
                must_update_counter_ = 4;
            }
        }
    }

    if (must_update_counter_ > 0)
    {
        must_update_counter_--;
        set_to_mujoco = true;
    }
    

    if (set_to_mujoco)
    {
        for (std::size_t dof = 0; dof < NB_DOFS; dof++)
        {
            o80::State2d state = set_states_.get(dof);
            if ((!std::isnan(state.get<0>())) && (!std::isnan(state.get<1>())))
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
