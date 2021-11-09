#pragma once

#include "mujoco.h"
#include "shared_memory/serializer.hpp"

namespace pam_mujoco
{
namespace internal
{
/**
 * Encapsulate the state (i.e. position and velocity)
 * of a ball and another object (i.e. racket and table)
 */
class ContactStates
{
public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(contactee_position,
                contactee_orientation,
                contactee_velocity,
                ball_position,
                ball_velocity,
                time_stamp);
    }

public:
    ContactStates();

public:
    // contactee: likely to be a racket or a table
    std::array<double, 9> contactee_orientation;
    std::array<double, 3> contactee_position;
    std::array<double, 3> contactee_velocity;
    std::array<double, 3> ball_position;
    std::array<double, 3> ball_velocity;
    double time_stamp;
};

/**
 * extract information from d to update the instance
 * of get_states. position of the ball, velocity of the ball,
 * position of the contactee and orientation of the contactee
 * are directly read from d.
 * the velocity of the contactee is computed via finite difference
 * using the position of the contactee as provided by get_states.
 * (i.e. get_states is also expected to encapsulte the previous
 * state).
 */
void save_state(const mjData* d,
                int index_qpos,
                int index_qvel,
                int index_geom_contactee,
                internal::ContactStates& get_states);

}  // namespace internal

}  // namespace pam_mujoco
