#pragma once

#include "o80/back_end.hpp"
#include "o80/memory_clearing.hpp"
#include "o80/state2d.hpp"
#include "o80/time.hpp"
#include "o80_pam/robot_fk_extended_state.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{
template <int QUEUE_SIZE, int NB_DOFS>
class MirrorRobot : public ControllerBase
{
private:
    typedef o80::BackEnd<QUEUE_SIZE,
                         NB_DOFS,
                         o80::State2d,
                         o80_pam::RobotFKExtendedState>
        Backend;
    typedef o80::States<NB_DOFS, o80::State2d> States;

public:
    MirrorRobot(std::string segment_id,
                std::string robot_joint_base,
                std::string robot_racket);
    void apply(const mjModel* m, mjData* d);

public:
    static void clear(std::string segment_id);

private:
    bool same(const States& s1, const States& s2) const;
    void update_robot_fk(const mjData* d);

private:
    Backend backend_;
    std::string robot_joint_base_;
    std::string robot_racket_;
    int index_geom_;
    int index_q_robot_;
    int index_qvel_robot_;
    States read_states_;
    States set_states_;
    States previous_set_states_;
    o80_pam::RobotFKExtendedState robot_fk_;
    int must_update_counter_ = -1;              // only overwrite if new robot state
};  

#include "mirror_robot.hxx"

}  // namespace pam_mujoco
