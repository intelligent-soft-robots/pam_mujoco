
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
}

template <int QUEUE_SIZE, int NB_DOFS>
bool MirrorRobot<QUEUE_SIZE, NB_DOFS>::same(const States& s1,
                                            const States& s2) const
{
    for (std::size_t dof = 0; dof < NB_DOFS; dof++)
    {
        o80::State2d t1 = s1.get(dof);
        o80::State2d t2 = s2.get(dof);
        //if (t1.get<0>() != t2.get<0>())
        if (std::abs(t1.get<0>() - t2.get<0>()) > 1e-5)
        {
            return false;
        }
        //if (t1.get<1>() != t2.get<1>())
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
    for (int dim = 0; dim < 3; dim++)
    {
        robot_fk_.set_position(dim, d->geom_xpos[index_geom_ * 3 + dim]);
    }
    for (int dim = 0; dim < 9; dim++)
    {
        robot_fk_.set_orientation(dim, d->geom_xmat[index_geom_ * 9 + dim]);
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
        // printf("must update robot\n");

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

        
        for(int dof = 0; dof < NB_DOFS; dof++)
        {
            o80::State2d state = set_states_.get(dof);
            // printf("state %d: %f %f  ",dof,state.get<0>(),state.get<1>());
            
        }
            
        // printf("\n");

        // updating mujoco only if there is a change of desired state
        if (!same(set_states_, previous_set_states_))
        {
            // there is a new mujoco iteration, and set_states_
            // has also been updated, so mujoco has to mirror this
            // printf("set_to_muoco\n");
            must_update_counter_ = 4;
            // set_to_mujoco = true;
            previous_set_states_ = set_states_;

            

            
        }
        // check if all velocities  close to zero
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
                // printf("all velocities zero\n");
                must_update_counter_ = 4;
            }
        }
    }
    else
    {
        // still working on previous iteration, so mujoco
        // needs update, possibly resetting the same values
        // again
        // set_to_mujoco = true;
        // printf("no must update robot\n");
    }

    if (must_update_counter_ > 0)
    {
        must_update_counter_--;
        set_to_mujoco = true;
        // printf("must update counter: %d\n", must_update_counter_);
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
        // printf("overriding robot state pos: %f %f %f %f, vel: %f %f %f %f\n",
            // d->qpos[index_q_robot_ + 0], d->qpos[index_q_robot_ + 1], d->qpos[index_q_robot_ + 2], d->qpos[index_q_robot_ + 3],
            // d->qvel[index_qvel_robot_ + 0], d->qvel[index_qvel_robot_ + 1], d->qvel[index_qvel_robot_ + 2], d->qvel[index_qvel_robot_ + 3]);
    }
}

template <int QUEUE_SIZE, int NB_DOFS>
void MirrorRobot<QUEUE_SIZE, NB_DOFS>::clear(std::string segment_id)
{
    o80::clear_shared_memory(segment_id);
}
