#pragma once

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
    void serialize(Archive &archive)
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

}  // namespace internal

}  // namespace pam_mujoco
