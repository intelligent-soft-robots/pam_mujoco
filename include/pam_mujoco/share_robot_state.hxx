ShareRobotState::ShareRobotState(std::string segment_id,
                                 std::string robot_joint_base)
    : joint_states_{segment_id, 4, true, true},
      robot_joint_base_(robot_joint_base),
      index_q_robot_(-1),
      index_qvel_robot_(-1)
{
}

void ShareRobotState::apply(const mjModel* m, mjData* d)
{
    if (index_q_robot_ < 0)
    {
        index_q_robot_ = m->jnt_qposadr[mj_name2id(
            m, mjOBJ_JOINT, robot_joint_base_.c_str())];
        index_qvel_robot_ = m->jnt_dofadr[mj_name2id(
            m, mjOBJ_JOINT, robot_joint_base_.c_str())];
    }
    // joint_states_ is a shared memory array,
    // so values directly shared
    JointState js;
    for (std::size_t dof = 0; dof < 4; dof++)
    {
        js.set<0>(d->qpos[index_q_robot_ + dof]);
        js.set<1>(d->qvel[index_qvel_robot_ + dof]);
        joint_states_.set(dof, js);
    }
}

void ShareRobotState::clear(std::string segment_id)
{
    shared_memory::clear_array(segment_id);
}
