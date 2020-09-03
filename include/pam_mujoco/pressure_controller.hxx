
template<int QUEUE_SIZE, int NB_DOFS>
PressureController<QUEUE_SIZE,
		   NB_DOFS>::PressureController(std::string segment_id,
						int scale_min_pressure, int scale_max_pressure,
						int scale_min_activation, int scale_max_activation)
		     : backend_(segment_id),
		       scale_min_pressure_(scale_min_pressure),
		       scale_min_activation_(scale_min_activation),
		       scale_ratio_((scale_max_pressure-scale_min_pressure)-
				    (scale_max_activation-scale_min_activation))
{}

template<int QUEUE_SIZE, int NB_DOFS>
void PressureController<QUEUE_SIZE,
			NB_DOFS>::PressureControllerapply(const mjModel* m,
							  mjData* d)
{
  for (std::size_t dof=0;dof<N_DOFS;dof++)
    {
      states_.values[dof*2].set(activation2pressure(d->act[dof*2]));
      states_.values[dof*2+1].set(activation2pressure(d->act[dof*2+1]));
    }
  const States& states = backend_.pulse(o80::time_now(),
					states_,
					pam_interface::RobotState<NB_DOFS>());
  for (std::size_t dof=0;dof<N_DOFS;dof++)
    {
      d->ctrl[dof*2]=pressure2activation(states.get(dof*2).get());
      d->ctrl[dof*2+1]=pressure2activation(states.get(dof*2+2).get());
    }
  states_ = states;
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

