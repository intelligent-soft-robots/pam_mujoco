

template<int QUEUE_SIZE, int NB_DOFS>
MirrorExternalRobot<QUEUE_SIZE,
		    NB_DOFS>::MirrorExternalRobot(std::string segment_id)
		      : backend_{segment_id},
			index_q_robot_(-1),
			index_qvel_robot_(-1)
{
}

template<int QUEUE_SIZE, int NB_DOFS>
void MirrorExternalRobot<QUEUE_SIZE,
		    NB_DOFS>::set_state(mjData* d)
{
  const States& states = backend_.pulse(o80::time_now(),
					states_,
					o80::VoidExtendedState());
  for(std::size_t dof=0; dof<NB_DOFS; dof++)
    {
      o80::State2d state = states.get(dof);
      d->qpos[index_q_robot_+dof] = state.get<0>();
      d->qvel[index_qvel_robot_+dof]= state.get<1>();
    }
  states_ = states;
}

template<int QUEUE_SIZE, int NB_DOFS>
void MirrorExternalRobot<QUEUE_SIZE,
		    NB_DOFS>::apply(const mjModel* m,
				    mjData* d)
{
  if(index_q_robot_<0)
    {
      index_q_robot_ = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "joint_base_rotation")];
      index_qvel_robot_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "joint_base_rotation")];
      o80::State2d joint_state;
      for(std::size_t dof=0; dof<NB_DOFS; dof++)
	{
	  joint_state.set<0>(d->qpos[index_q_robot_+dof]);
	  joint_state.set<1>(d->qvel[index_qvel_robot_+dof]);
	  states_.set(dof,joint_state);
	}
    }
  set_state(d);
}

template<int QUEUE_SIZE, int NB_DOFS>
void MirrorExternalRobot<QUEUE_SIZE,
			 NB_DOFS>::clear(std::string segment_id)
{
  o80::clear_shared_memory(segment_id);
}
