

template<int QUEUE_SIZE, int NB_DOFS>
MirrorExternalRobot<QUEUE_SIZE,
		    NB_DOFS>::MirrorExternalRobot(std::string segment_id,
						  mjData* d_init)
		      : backend_{segment_id}
{
  pam_interface::JointState joint_state;
  for(std::size_t dof=0; dof<NB_DOFS; dof++)
    {
      joint_state.position = d_init[index_q_robot_+dof];
      joint_state.velocity = d_init[index_qvel_robot_+dof];
      states_.set(dof,joint_state);
    }
  index_q_robot_ = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "joint_base_rotation")];
  index_qvel_robot_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "joint_base_rotation")];
}

template<int QUEUE_SIZE, int NB_DOFS>
void MirrorExternalRobot<QUEUE_SIZE,
		    NB_DOFS>::set_state(mjData* d)
{
  States states = backend_.pulse(o80::TimePoint(0),
				 states_,
				 o80::VoidExtendedState());
  for(std::size_t dof=0; dof<NB_DOFS; dof++)
    {
      d[index_q_robot_+dof] = states.get(dof).position;
      d[index_qvel_robot_+dof]= states.get(dof).velocity;
    }
  states_ = states;
}

template<int QUEUE_SIZE, int NB_DOFS>
void MirrorExternalRobot<QUEUE_SIZE,
		    NB_DOFS>::control(const mjModel* m,
				      mjData* d)
{
  set_state(d);
}

template<int QUEUE_SIZE, int NB_DOFS>
void MirrorExternalRobot<QUEUE_SIZE,
			 NB_DOFS>::clear(std::string segment_id)
{
  o80::clear_shared_memory(segment_id);
}
