#include "pam_mujoco/internal/contact_states.hpp"

namespace pam_mujoco
{
namespace internal
{
ContactStates::ContactStates() : time_stamp(-1)
{
}

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
                internal::ContactStates& get_states)
{
    // velocity of contactee computed with finite differences
    if (get_states.time_stamp < 0)
    {
        for (size_t i = 0; i < 3; i++) get_states.contactee_velocity[i] = 0;
    }
    else
    {
        double delta_time = d->time - get_states.time_stamp;
        if (delta_time != 0)
        {
            for (size_t i = 0; i < 3; i++)
            {
                get_states.contactee_velocity[i] =
                    (d->geom_xpos[index_geom_contactee * 3 + i] -
                     get_states.contactee_position[i]) /
                    delta_time;
            }
        }
    }
    // rest is just copied from d to get_states
    for (size_t i = 0; i < 3; i++)
    {
        get_states.ball_position[i] = d->qpos[index_qpos + i];
        get_states.ball_velocity[i] = d->qvel[index_qvel + i];
        get_states.contactee_position[i] =
            d->geom_xpos[index_geom_contactee * 3 + i];
    }
    for (size_t i = 0; i < 9; i++)
    {
        get_states.contactee_orientation[i] =
            d->geom_xmat[index_geom_contactee * 9 + i];
    }
    get_states.time_stamp = d->time;
}

}  // namespace internal

}  // namespace pam_mujoco
