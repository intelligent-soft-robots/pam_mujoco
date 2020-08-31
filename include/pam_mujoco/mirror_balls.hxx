

template<int QUEUE_SIZE, int NB_BALLS>
MirrorBalls<QUEUE_SIZE,
	    NB_BALLS>::MirrorBalls(std::string segment_id,
				   std::string ball_obj_joint)
		      : backend_{segment_id},
			ball_obj_joint_(ball_obj_joint),
			index_q_balls_(-1),
			index_qvel_balls_(-1)
{
  contact_interrupts_.fill(false);
  interrupted_.fill(false);
}

template<int QUEUE_SIZE, int NB_BALLS>
void MirrorBalls<QUEUE_SIZE,
		    NB_BALLS>::set_state(mjData* d)
{
  // saving current mujoco state in states_
  for(std::size_t ball=0; ball<NB_BALLS; ball++)
    {
      int index_mujoco = ball*3;
      int index_state = ball*6;
      states_.values[index_state].value
	= d->qpos[index_q_balls_+index_mujoco];
      states_.values[index_state+2].value
	= d->qpos[index_q_balls_+index_mujoco+1];
      states_.values[index_state+4].value
	= d->qpos[index_q_balls_+index_mujoco+2];
      states_.values[index_state+1].value
	= d->qvel[index_qvel_balls_+index_mujoco];
      states_.values[index_state+3].value
	= d->qvel[index_qvel_balls_+index_mujoco+1];
      states_.values[index_state+5].value
	= d->qvel[index_qvel_balls_+index_mujoco+1];
    }

  const States& states = backend_.pulse(o80::time_now(),
					states_,
					o80::VoidExtendedState());

  // some balls have been set to have the trajectory interrupted
  // in case of contact (i.e. either customed model or mujoco engine
  // take hand). Checking if such contact occured.
  // (note: see Contacts.hpp to see what serialize ContactInformation
  // instances into the shared memory)
  ContactInformation ci;
  for(std::size_t ball=0; ball<NB_BALLS; ball++)
    {
      if( contact_interrupts_[ball] && !interrupted_[ball] )
	{
	  shared_memory::deserialize(segment_id_contacts_[ball],
				    segment_id_contacts_[ball],
				    ci);
	  if(ci.contact_occured)
	    {
	      interrupted_[ball]=true;
	    }
	}
    }
  
  for(std::size_t ball=0; ball<NB_BALLS; ball++)
    {
      // mirroring only if a contact did not
      // interrupt mirroring
      if(! interrupted_[ball])
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
    }
}




template<int QUEUE_SIZE, int NB_BALLS>
void MirrorBalls<QUEUE_SIZE,
		 NB_BALLS>::set_contact_interrupt(int ball_index,
						  std::string segment_id)
{
  contact_interrupts_[ball_index]=true;
  segment_id_contacts_[ball_index]=segment_id;
}
  


template<int QUEUE_SIZE, int NB_BALLS>
void MirrorBalls<QUEUE_SIZE,
		    NB_BALLS>::apply(const mjModel* m,
				    mjData* d)
{
  if(index_q_balls_<0)
    {
      index_q_balls_ = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT,
						 ball_obj_joint_.c_str())];
      index_qvel_balls_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT,
						   ball_obj_joint_.c_str())];
    }
  set_state(d);
}

template<int QUEUE_SIZE, int NB_BALLS>
void MirrorBalls<QUEUE_SIZE,
			 NB_BALLS>::clear(std::string segment_id)
{
  o80::clear_shared_memory(segment_id);
}
