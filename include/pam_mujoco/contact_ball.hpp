#pragma once

#include "context/contact_information.hpp"
#include "mujoco.h"
#include "pam_mujoco/internal/contact_logic.hpp"
#include "pam_mujoco/internal/contact_states.hpp"
#include "pam_mujoco/recompute_state_after_contact.hpp"
#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"

#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{
void save_state(const mjData* d,
                int index_q_ball,
                int index_qvel_ball,
                int index_geom_ball,
                int index_geom_contactee,
                internal::ContactStates& get_states,
                bool verbose);

/**
 * controller for managing the contact between the ball
 * and a contactee (i.e. racket or table), i.e.
 * detecting contacts and recomputing resulting ball
 * position and velocity
 */
class ContactBall : public ControllerBase
{
public:
    ContactBall(std::string segment_id_,
                int index_qpos,
                int index_qvel,
                std::string geom,
                std::string geom_contactee,
                ContactItems contact_item);

public:
    // ControllerBase function
    void apply(const mjModel* m, mjData* d);

private:
    void init(const mjModel* m);
    void reset();

private:
    std::string segment_id_;
    RecomputeStateConfig config_;
    context::ContactInformation contact_information_;
    internal::ContactStates previous_;
    int index_qpos_;
    int index_qvel_;
    std::string geom_;
    std::string geom_contactee_;
    int index_geom_;
    int index_geom_contactee_;  // contactee : racket or table
    internal::ContactLogic contact_logic_;
};

void activate_contact(const std::string& segment_id);
void deactivate_contact(const std::string& segment_id);
void reset_contact(const std::string& segment_id);

}  // namespace pam_mujoco
