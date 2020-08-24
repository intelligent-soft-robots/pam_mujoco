

template<int QUEUE_SIZE, int NB_BALLS>
MirrorExternalBalls<QUEUE_SIZE,
		    NB_BALLS>::MirrorExternalBalls(std::string segment_id,
						  const mjModel* m,
						  const mjData* d_init)
		      : backend_{segment_id}
{
  index_q_balls_ = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "ball_free_jnt")];
  index_qvel_balls_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "ball_free_jnt")];
}

template<int QUEUE_SIZE, int NB_BALLS>
void MirrorExternalBalls<QUEUE_SIZE,
		    NB_BALLS>::set_state(mjData* d)
{
  const States& states = backend_.pulse(o80::time_now(),
					states_,
					o80::VoidExtendedState());
  for(std::size_t ball=0; ball<NB_BALLS; ball++)
    {
      int index_mujoco = ball*3;
      int index_state = ball*6;
      // x,y,z positions
      d->qpos[index_q_balls_+index_mujoco]
	= states.get(index_state).value;
      d->qpos[index_q_balls_+index_mujoco+1]
	= states.get(index_state+2).value;
      d->qpos[index_qvel_balls_+index_mujoco+2]
	= states.get(index_state+4).value;
      // x,y,z velocities
      d->qvel[index_qvel_balls_+index_mujoco]
	= states.get(index_state+1).value;
      d->qvel[index_qvel_balls_+index_mujoco+1]
	= states.get(index_state+3).value;
      d->qvel[index_q_balls_+index_mujoco+2]
      = states.get(index_state+5).value;
    }
  states_ = states;
}

template<int QUEUE_SIZE, int NB_BALLS>
void MirrorExternalBalls<QUEUE_SIZE,
		    NB_BALLS>::apply(const mjModel* m,
				    mjData* d)
{
  set_state(d);
}

template<int QUEUE_SIZE, int NB_BALLS>
void MirrorExternalBalls<QUEUE_SIZE,
			 NB_BALLS>::clear(std::string segment_id)
{
  o80::clear_shared_memory(segment_id);
}
