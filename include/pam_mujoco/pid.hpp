#pragma once

#include <vector>
#include <algorithm>
#include <Eigen/Dense>

namespace pam_mujoco
{

  class PID
  {
    
  public:
    
    PID(std::vector<double> kp,
	std::vector<double> kd,
	std::vector<double> ki);
    
    void set_desired(const std::vector<double>& desired);

    void raw_control(const Eigen::VectorXd position,
		     Eigen::VectorXd ctrl,
		     double time);
    
    void pam_control(const double *position, double time,
		     double *ctrl);
    
  private:

    int nb_dofs_;
    double previous_time_;
    Eigen::VectorXd desired_;
    Eigen::VectorXd error_sum_;
    Eigen::VectorXd previous_error_;
    Eigen::VectorXd u_;
    Eigen::MatrixXd kp_;
    Eigen::MatrixXd ki_;
    Eigen::MatrixXd kd_;
    
  };
  
}
