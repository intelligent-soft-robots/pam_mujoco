#pragma once

#include <array>
#include <map>
#include "context/contact_information.hpp"
#include "o80/back_end.hpp"
#include "o80/item3d_state.hpp"
#include "o80/memory_clearing.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/extra_balls_extended_state.hpp"
#include "pam_mujoco/joint_state.hpp"

namespace pam_mujoco
{
template <int QUEUE_SIZE, int NB_ITEMS>
class MirrorFreeJoints : public ControllerBase
{
private:
    typedef o80::BackEnd<QUEUE_SIZE,
                         NB_ITEMS,
                         o80::Item3dState,
                         ExtraBallsExtendedState<NB_ITEMS>>
        Backend;
    typedef o80::States<NB_ITEMS, o80::Item3dState> States;

public:
    MirrorFreeJoints(std::string mujoco_id,
                     std::string segment_id,
                     std::array<std::string, NB_ITEMS> joint,
                     std::array<int, NB_ITEMS> index_qpos,
                     std::array<int, NB_ITEMS> index_qvel,
                     std::string geom_robot,
                     bool active_only);
    MirrorFreeJoints(std::string mujoco_id,
                     std::string segment_id,
                     std::array<std::string, NB_ITEMS> joint,
                     std::array<int, NB_ITEMS> index_qpos,
                     std::array<int, NB_ITEMS> index_qvel,
                     std::string geom_robot,
                     std::array<std::string, NB_ITEMS> interrupt_segment_id,
                     bool active_only);
    void set_contact_interrupt(int item_index, std::string segment_id);
    void apply(const mjModel* m, mjData* d);

public:
    static void clear(std::string segment_id);

private:
    std::string mujoco_id_;
    std::string segment_id_;
    std::array<std::string, NB_ITEMS> joint_;
    std::array<int, NB_ITEMS> index_qpos_;
    std::array<int, NB_ITEMS> index_qvel_;
    std::string geom_robot_;
    int index_robot_geom_;
    Backend backend_;
    States read_states_;
    States set_states_;

    std::array<bool, NB_ITEMS> contact_interrupt_;
    std::array<bool, NB_ITEMS> interrupted_;
    std::array<std::string, NB_ITEMS> segment_id_contact_;

    bool active_only_;
};

#include "mirror_free_joints.hxx"

}  // namespace pam_mujoco
