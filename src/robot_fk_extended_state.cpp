#include "pam_mujoco/robot_fk_extended_state.hpp"

namespace pam_mujoco
{
  RobotFKExtendedState::RobotFKExtendedState(){}

  RobotFKExtendedState::RobotFKExtendedState(const std::array<double,3>& position,
					     const std::array<double,3>& velocity,
					     const std::array<double,9>& orientation)
    : position_{position},
      velocity_{velocity},
      orientation_{orientation}
  {}

  const std::array<double,3>& RobotFKExtendedState::get_position()
  {
    return position_;
  }

  const std::array<double,3>& RobotFKExtendedState::get_velocity()
  {
    return velocity_;
  }
  
  const std::array<double,9>& RobotFKExtendedState::get_orientation()
  {
    return orientation_;
  }

  void RobotFKExtendedState::set_position(int dim, double value)
  {
    position_[dim]=value;
  }
  
  void RobotFKExtendedState::set_velocity(int dim, double value)
  {
    velocity_[dim]=value;
  }
  
  void RobotFKExtendedState::set_orientation(int dim, double value)
  {
    orientation_[dim]=value
  }

}
