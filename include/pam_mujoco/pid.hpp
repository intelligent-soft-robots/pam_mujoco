
#pragma once

#include <vector>
#include <algorithm>

namespace pam_mujoco
{

  class PID
  {
    
  public:
    
    PID(std::vector<double> kp,
	std::vector<double> kd,
	std::vector<double> ki);
    
    void set_desired(const std::vector& desired);
    void control(const double *position, double time,
		 double *ctrl);
    
  private:
    
   static double control_(double kp, double kd, double ki,
			  double position,
			  double desired,
			  double delta_time,
			  double& previous_error,
			  double& error_sum)
  private:
    
    int nb_dofs;
    double previous_time_;
    std::vector<double> desired_;
    std::vector<double> kp_;
    std::vector<double> kd_;
    std::vector<double> ki_;
    std::vector<double> error_sum_;
    std::vector<double> previous_error_;
    std::vector<double> u_;
    
  };
  
}
