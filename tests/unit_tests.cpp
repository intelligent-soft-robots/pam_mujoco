#include "gtest/gtest.h"

#include "pam_mujoco/pid.hpp"


class PamMujocoTests : public ::testing::Test
{
  void SetUp(){}
  void TearDown(){}
};


TEST_F(PamMujocoTests,pid_at_ref)
{
  std::vector<double> kp = {1,1,1};
  std::vector<double> ki = {1,1,1};
  std::vector<double> kd = {1,1,1};

  pam_mujoco::PID pid{kp,kd,ki};

  std::vector<double> desired = {2,2,2};
  pid.set_desired(desired);

  Eigen::Vector3d position(2,2,2);
  Eigen::Vector3d ctrl;
  double time = 0;
  
  pid.raw_control(position,ctrl,time);
  for(int dim=0;dim<3;dim++)
    {
      ASSERT_EQ(ctrl(dim),0);
    }
  
  time+=1;

  pid.raw_control(position,ctrl,time);
  for(int dim=0;dim<3;dim++)
    {
      ASSERT_EQ(ctrl(dim),0);
    }
  
}



