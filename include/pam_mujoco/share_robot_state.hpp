#pragma once

#include "o80/memory_clearing.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{
class ShareRobotState : public ControllerBase
{
public:
    ShareRobotState(std::string segment_id, std::string robot_joint_base);
    void apply(const mjModel* m, mjData* d);

public:
    static void clear(std::string segment_id);

private:
    shared_memory::array<JointState> joint_states_;
    std::string robot_joint_base_;
    int index_q_robot_;
    int index_qvel_robot_;
};

#include "share_robot_state.hxx"

}  // namespace pam_mujoco
