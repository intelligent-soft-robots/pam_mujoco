#include "pam_mujoco/pid.hpp"

namespace pam_mujoco
{

  PID::PID(std::vector<double> kp,
	   std::vector<double> kd,
	   std::vector<double> ki)
    : nb_dofs_(kp.size()),
	previous_time_(-1),
	kp_(kp),kd_(kd),ki_(ki)
    {
      desired_.resize(nb_dofs_);
      error_sum_.resize(nb_dofs_);
      previous_error_.resize(nb_dofs);
      std::fill(desired_.begin(), desired_.end(), 0);
      std::fill(error_sum_.begin(), error_sum_.end(), 0);
      std::fill(previous_error_.begin(), previous_error_.end(), 0);
      u_.resize(nb_dofs_);
    }

  void PID::set_desired(const std::vector& desired)
    {
      desired_ = desired;
    }
  
  void PID::control(const double *position, double time,
		    double *ctrl)
  {
    if(previous_time_<0)
      previous_time_ = time;
    double delta_time = time - previous_time_;
    for(size_t dof=0;dof<nb_dofs;dof++)
      {
	double u = control_(kp[dof],kd[dof],ki[dof],
			    position[dof],desired_[dof],
			    delta_time,previous_error_[dof],
			    error_sum_[dof]);
	if(u<0)
	  {
	    ctrl[dof*2]= std::abs(u)+0.1;
	    ctrl[dof*2+1]= 0.1;
	  }
	else
	  {
	    ctrl[dof*2+1]= std::abs(u)+0.1;
	    ctrl[dof*2]= 0.1;
	  }
	previous_time_ = time;
      }
  }


  double PID::control_(double kp, double kd, double ki,
		       double position,
		       double desired,
		       double delta_time,
		       double& previous_error,
		       double& error_sum)
  {
    double error = desired-position;
    double error_d = (error-previous_error)/delta_time;
    error_sum += error*delta_time;
    
    double u =
      (error*kp)+
      error_sum*ki+
      error_d*kd;
    
    previous_error = error;
    
    return u;
    
  }
