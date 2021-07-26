// Copyright (c) 2021 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <array>
#include "shared_memory/serializer.hpp"

namespace pam_mujoco
{
  /* ! RobotFKExtendedState encapsulates the forward kinematics of the robot
   *   (position, velocity and orientation of the racket).
   *   It is meant to be used as ExtendedState in a o80 observation 
   *   (i.e. supplementary data that can be added to an observation).
   *   See pam_mujoco/mirror_robot.hpp for an example of the FK data of the 
   *   simulated robot is added to an (o80) observation
   */

  class RobotFKExtendedState
  {
  public:
    RobotFKExtendedState();
    RobotFKExtendedState(const std::array<double,3>& position,
			 const std::array<double,3>& velocity,
			 const std::array<double,9>& orientation);
    void set_position(int dim, double value);
    void set_velocity(int dim, double value);
    void set_orientation(int dim, double value);
    const std::array<double,3>& get_position();
    const std::array<double,3>& get_velocity();
    const std::array<double,9>& get_orientation();
    template <class Archive>
    void serialize(Archive& archive)
    {
      archive(position_,velocity_,orientation_);
    }
  private:
    friend shared_memory::private_serialization;
    std::array<double,3> position_;
    std::array<double,3> velocity_;
    std::array<double,9> orientation_;
  };
  


}


