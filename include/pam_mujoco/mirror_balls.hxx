

template<int QUEUE_SIZE, int NB_BALLS>
MirrorBalls<QUEUE_SIZE,
		    NB_BALLS>::MirrorBalls(std::string segment_id)
		      : backend_{segment_id},
			index_q_balls_(-1),
			index_qvel_balls_(-1)
{
}

template<int QUEUE_SIZE, int NB_BALLS>
void MirrorBalls<QUEUE_SIZE,
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
void MirrorBalls<QUEUE_SIZE,
		    NB_BALLS>::apply(const mjModel* m,
				    mjData* d)
{
  if(index_q_balls_<0)
    {
      index_q_balls_ = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "ball_free_jnt")];
      index_qvel_balls_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "ball_free_jnt")];
    }
  set_state(d);
}

template<int QUEUE_SIZE, int NB_BALLS>
void MirrorBalls<QUEUE_SIZE,
			 NB_BALLS>::clear(std::string segment_id)
{
  o80::clear_shared_memory(segment_id);
}
