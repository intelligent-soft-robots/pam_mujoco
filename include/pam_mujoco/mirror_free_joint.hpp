#pragma once

#include <map>
#include "context/contact_information.hpp"
#include "o80/back_end.hpp"
#include "o80/memory_clearing.hpp"
#include "o80/state1d.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{
template <int QUEUE_SIZE>
class MirrorFreeJoint : public ControllerBase
{
private:
    typedef o80::BackEnd<QUEUE_SIZE,
                         6,  // 6: 3d position and 3d velocity per ball
                         o80::State1d,
                         o80::VoidExtendedState>
        Backend;
    typedef o80::States<6, o80::State1d> States;

public:
    MirrorFreeJoint(std::string segment_id,
                    std::string joint,
                    int index_qpos,
                    int index_qvel,
                    bool active_only = true);
    MirrorFreeJoint(std::string segment_id,
                    std::string joint,
                    int index_qpos,
                    int index_qvel,
                    std::string interrupt_segment_id,
                    bool active_only = true);
    void set_contact_interrupt(std::string segment_id);
    void apply(const mjModel* m, mjData* d);

public:
    static void clear(std::string segment_id);

private:
    Backend backend_;
    std::string segment_id_;
    std::string joint_;
    int index_qpos_;
    int index_qvel_;
    bool contact_interrupt_;
    bool interrupted_;
    bool active_only_;
    States read_states_;
    States set_states_;
    std::string segment_id_contact_;
};

#include "mirror_free_joint.hxx"

}  // namespace pam_mujoco
