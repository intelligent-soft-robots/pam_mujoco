#include "pam_mujoco/read_robot_state.hpp"

namespace pam_mujoco
{
ReadRobotState::ReadRobotState(std::string segment_id)
    : joint_states_{segment_id, 4, true, true}
{
}

const std::array<double, 4>& ReadRobotState::get_positions()
{
    JointState js;
    for (std::size_t dof = 0; dof < 4; dof++)
    {
        joint_states_.get(dof, js);
        positions_[dof] = js.get<0>();
    }
    return positions_;
}

const std::array<double, 4>& ReadRobotState::get_velocities()
{
    JointState js;
    for (std::size_t dof = 0; dof < 4; dof++)
    {
        joint_states_.get(dof, js);
        velocities_[dof] = js.get<1>();
    }
    return velocities_;
}

}  // namespace pam_mujoco
