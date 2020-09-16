

template<int QUEUE_SIZE, int NB_DOFS>
PressureController<QUEUE_SIZE,
		   NB_DOFS>::PressureController(std::string segment_id,
						int scale_min_pressure, int scale_max_pressure,
						int scale_min_activation, int scale_max_activation,
						std::string muscle_json_config_path_ago,
						std::string muscle_json_config_path_antago,
						std::array<double,NB_DOFS*2> a_init,
						std::array<double,NB_DOFS*2> l_MTC_change_init)
		     : backend_(segment_id),
		       scale_min_pressure_(scale_min_pressure),
		       scale_min_activation_(scale_min_activation),
		       scale_ratio_((scale_max_pressure-scale_min_pressure)-
				    (scale_max_activation-scale_min_activation))
{
  for(std::size_t dof=0;dof<NB_DOFS;dof++)
    {
      muscles_.push_back(pam_models::hill::from_json(muscle_json_config_path_ago,
						    a_init[dof*2],
						    l_MTC_change_init[dof*2]));
      muscles_.push_back(pam_models::hill::from_json(muscle_json_config_path_antago,
						    a_init[dof*2+1],
						    l_MTC_change_init[dof*2+1]));
    }
}

template<int QUEUE_SIZE, int NB_DOFS>
void PressureController<QUEUE_SIZE,
			NB_DOFS>::apply(const mjModel* m,
					mjData* d)
{
  // current state, for input to o80 (i.e. observation generation)
  States current_states;
  for (std::size_t dof=0;dof<NB_DOFS;dof++)
    {
      current_states.values[dof*2].set(activation2pressure(d->act[dof*2]));
      current_states.values[dof*2+1].set(activation2pressure(d->act[dof*2+1]));
    }
  // reading current robot state
  
  // reading desired pressure from o80
  const States& states = backend_.pulse(o80::time_now(),
					current_states,
					pam_interface::RobotState<NB_DOFS>());
  for (std::size_t dof=0;dof<NB_DOFS;dof++)
    {
      d->ctrl[dof*2]=pressure2activation(states.get(dof*2).get());
      d->ctrl[dof*2+1]=pressure2activation(states.get(dof*2+1).get());
    }
  // applying them to the muscles
  for (std::size_t muscle=0;muscle<NB_DOFS*2;muscle++)
    {
      int index = 2*NB_DOFS+muscle;
      double l_MTC = d->actuator_length[index];
      double dot_l_MTC = d->actuator_velocity[index];
      double a = d->act[muscle];
      double l_CE = d->act[index];
      std::tuple<double,double> r = muscles_[muscle].get(l_MTC,
							 dot_l_MTC,
							 a,l_CE);
      d->ctrl[NB_DOFS*2+muscle] = std::get<1>(r);
      // bias_forces_ will be used by the get_bias method below
      bias_forces_[muscle] = std::get<0>(r);
    }
}

template<int QUEUE_SIZE, int NB_DOFS>
mjtNum PressureController<QUEUE_SIZE,
			  NB_DOFS>::get_bias(const mjModel* m,
					     const mjData* d,
					     int id)
{
  int i_muscle = id - 2*NB_DOFS; 
  return -bias_forces_[i_muscle]; // computed in the apply function
  // note: optional filtering performed in the original code, ignored for now 
}

template<int QUEUE_SIZE, int NB_DOFS>
double PressureController<QUEUE_SIZE,
			  NB_DOFS>::PressureController::pressure2activation(double pressure)
{
  return (pressure-scale_min_pressure_)
    / scale_ratio_ + scale_min_activation_;
}

template<int QUEUE_SIZE, int NB_DOFS>
double PressureController<QUEUE_SIZE,
			  NB_DOFS>::PressureController::activation2pressure(double activation)
{
  return (activation - scale_min_activation_)
    * scale_ratio_ + scale_min_pressure_;
}

template<int QUEUE_SIZE, int NB_DOFS>
void PressureController<QUEUE_SIZE,
			NB_DOFS>::clear(std::string segment_id)
{
  o80::clear_shared_memory(segment_id);
}
