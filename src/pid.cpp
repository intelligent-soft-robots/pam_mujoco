#include "pam_mujoco/pid.hpp"

namespace pam_mujoco
{

  PID::PID(std::vector<double> kp,
	   std::vector<double> kd,
	   std::vector<double> ki)
    : nb_dofs_(kp.size()),
      previous_time_(-1),
      desired_(Eigen::VectorXd::Zero(nb_dofs_)),
      error_sum_(Eigen::VectorXd::Zero(nb_dofs_)),
      previous_error_(Eigen::VectorXd::Zero(nb_dofs_)),
      u_(Eigen::VectorXd::Zero(nb_dofs_)),
      kp_(Eigen::MatrixXd::Zero(nb_dofs_,nb_dofs_)),
      ki_(Eigen::MatrixXd::Zero(nb_dofs_,nb_dofs_)),
      kd_(Eigen::MatrixXd::Zero(nb_dofs_,nb_dofs_))
    {
      for(uint dof=0;dof<nb_dofs_;dof++)
	{
	  kp_(dof,dof)=kp[dof];
	  ki_(dof,dof)=ki[dof];
	  kd_(dof,dof)=kd[dof];
	}
    }

  void PID::set_desired(const std::vector<double>& desired)
    {
      for(std::size_t dof=0;dof<nb_dofs_;dof++)
	{
	  desired_(dof)=desired[dof];
	}
    }

  void PID::raw_control(const Eigen::VectorXd position,
			Eigen::VectorXd ctrl,
			double time)
  {
    if(previous_time_<0)
      previous_time_ = time;
    double delta_time = time - previous_time_;
    Eigen::VectorXd error = desired_ - position;
    Eigen::VectorXd error_d = (error - previous_error_) / delta_time;
    error_sum_ = error_sum_ + error * delta_time;
    ctrl = kp_ *  error + ki_ * error_sum_ + kd_ * error_d;
  }
    
  
  void PID::pam_control(const double *position, double time,
			double *ctrl)
  {

    Eigen::VectorXd e_position = Eigen::Map<const Eigen::VectorXd>(position,nb_dofs_);
    Eigen::VectorXd e_ctrl = Eigen::Map<Eigen::VectorXd>(ctrl,nb_dofs_);

    raw_control(e_position,e_ctrl,time);
    
    for(size_t dof=0;dof<nb_dofs_;dof++)
      {
	double *c = &e_ctrl[dof];
	if(*c<0)
	  {
	    ctrl[dof*2]= std::abs(*c)+0.1;
	    ctrl[dof*2+1]= 0.1;
	  }
	else
	  {
	    ctrl[dof*2+1]= std::abs(*c)+0.1;
	    ctrl[dof*2]= 0.1;
	  }
      }
    previous_time_ = time;
    
  }


}
