

template<int CONTROLLER_INDEX, int QUEUE_SIZE, int NB_DOFS>
MirrorExternalRobot<CONTROLLER_INDEX, QUEUE_SIZE,
		    NB_DOFS>::MirrorExternalRobot(std::string segment_id,
						  int index_q_robot,
						  int index_qvel_robot,
						  mjData* d_init)
		      : ControllerBase(CONTROLLER_INDEX),
			index_q_robot_{index_q_robot},
			index_qvel_robot_{index_qvel_robot},
			backend_{segment_id}
{
  pam_interface::JointState joint_state;
  for(std::size_t dof=0; dof<NB_DOFS; dof++)
    {
      joint_state.position = d_init[index_q_robot_+dof];
      joint_state.velocity = d_init[index_qvel_robot_+dof];
      states_.set(dof,joint_state);
    }
}

template<int CONTROLLER_INDEX, int QUEUE_SIZE, int NB_DOFS>
void MirrorExternalRobot<CONTROLLER_INDEX, QUEUE_SIZE,
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

template<int CONTROLLER_INDEX, int QUEUE_SIZE, int NB_DOFS>
void MirrorExternalRobot<CONTROLLER_INDEX, QUEUE_SIZE,
		    NB_DOFS>::control(const mjModel* m,
				      mjData* d)
{
  set_state(d);
}

template<int CONTROLLER_INDEX, int QUEUE_SIZE, int NB_DOFS>
void MirrorExternalRobot<CONTROLLER_INDEX, QUEUE_SIZE,
			 NB_DOFS>::clear(std::string segment_id)
{
  o80::clear_shared_memory(segment_id);
}
