#pragma once

#include "context/contact_information.hpp"
#include "pam_mujoco/contact_items.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/internal/contact_states.hpp"
#include "pam_mujoco/internal/is_in_contact.hpp"
#include "pam_mujoco/recompute_state_after_contact.hpp"
#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"

#define NB_ITERATIONS_CONTACT_ACTIVE 200
#define NB_ITERATIONS_CONTACT_MUTED 1000

namespace pam_mujoco
{
void save_state(const mjData* d,
                int index_q_ball,
                int index_qvel_ball,
                int index_geom_ball,
                int index_geom_contactee,
                internal::ContactStates& get_states,
                bool verbose);


class ContactMode
{
public:
    ContactMode();
    void init(int index_qpos, int index_qvel, int index_geom, int index_contactee_geom);
    void reset();
    void set_contact_overwrite(mjData* d, const double* ball_position, const double* ball_velocity);
    bool contact_active(const mjModel* m, mjData* d);
private:
    void apply_model(mjData* d);
private:
    int steps_since_contact_;
    int steps_since_overwrite_;
    int index_qpos_;
    int index_qvel_;
    int index_geom_;
    int index_geom_contactee_;
    std::array<double,3> ball_position_;
    std::array<double,3> ball_velocity_;
    double contact_time_;
    o80::Milliseconds mujoco_time_step_;
};

    
    
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
                std::string joint,
                std::string geom,
                std::string robot_base,
                std::string geom_contactee,
                ContactItems contact_item);

public:
    // ControllerBase function
    void apply(const mjModel* m, mjData* d);

private:
    void init(const mjModel* m);
    void reset();
    bool user_signals();
    void share_contact_info();
    bool no_apply(const mjData* d);
    void save_state(const mjData* m, internal::ContactStates& cs);
    void execute(const mjModel* m, mjData* d);

private:
    std::string segment_id_;
    ContactMode contact_mode_;
    internal::RecomputeStateConfig config_;
    context::ContactInformation contact_information_;
    internal::ContactStates current_;
    internal::ContactStates previous_;
    std::string joint_;
    int index_qpos_;
    int index_qvel_;
    std::string robot_base_;
    int index_robot_qpos_;
    std::string geom_;
    std::string geom_contactee_;
    int index_geom_;
    int index_geom_contactee_;  // contactee : racket or table
    double mujoco_detected_dist_;
    int steps_overwrite_remaining_ = -1;
    double overwrite_ball_position_[3];
    double overwrite_ball_velocity_[3];
};

void activate_contact(const std::string& segment_id);
void deactivate_contact(const std::string& segment_id);
void reset_contact(const std::string& segment_id);

}  // namespace pam_mujoco
