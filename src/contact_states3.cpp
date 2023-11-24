#include "pam_mujoco/internal/contact_states.hpp"

namespace pam_mujoco
{
namespace internal
{
ContactStates::ContactStates() : time_stamp(-1), velocity_time_stamp(-1)
{
}

bool should_update_velocity(internal::ContactStates& previous_state,
                            mjtNum* m,
                            double time,
                            double time_threshold = 0.1,
                            double position_threshold = 0.000001)
{
    for (size_t i = 0; i < 3; i++)
    {
        if (abs(previous_state.contactee_position[i] - m[i])>position_threshold)
        {
            return true;
        }
        else
        {
            printf("%.20f\n", abs(previous_state.contactee_position[i] - m[i]));
        }
    }
    if ((time - previous_state.velocity_time_stamp) >= time_threshold)
    {
        return true;
    }
    return false;
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
            // ! this is quite hacky.
            // When running learning_table_tennis_from_scratch, the positions of
            // the robot joints are updated at a lower frequency: during the
            // intermediate iterations, this thread "sees" an immobile racket,
            // i.e. a velocity of zero, which is incorrect.
            // "should_update_velocity" returns true only if the racket moved,
            // or if a time threshold passed.
            if (should_update_velocity(
                    get_states,
                    &(d->geom_xpos[index_geom_contactee * 3]),
                    d->time))
            {
                double dt = d->time - get_states.velocity_time_stamp;
                for (size_t i = 0; i < 3; i++)
                {
                    get_states.contactee_velocity[i] =
                        (d->geom_xpos[index_geom_contactee * 3 + i] -
                         get_states.contactee_position[i]) /
                        dt;
                }
                get_states.velocity_time_stamp = d->time;
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
