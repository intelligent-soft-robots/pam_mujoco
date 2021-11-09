#include "pam_mujoco/recompute_state_after_contact.hpp"

namespace pam_mujoco
{
namespace internal
{

RecomputeStateConfig::RecomputeStateConfig()
{
}

RecomputeStateConfig get_table_recompute_config()
{
    RecomputeStateConfig config;
    config.epsilon = {0.73, 0.73, 0.92};
    config.rotation_matrix_contactee = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    config.vel_plus.fill(0.);
    config.mirror_y = false;
    return config;
}

RecomputeStateConfig get_robot1_recompute_config()
{
    RecomputeStateConfig config;
    config.epsilon = {0.78, 0.78, 0.78};
    config.rotation_matrix_contactee = {
        0.000, -1.000, 0.000, -1.000, 0.000, 0.000, 0.000, 0.000, -1.000};
    config.vel_plus.fill(0.);
    config.mirror_y = true;
    return config;
}

RecomputeStateConfig get_robot2_recompute_config()
{
    RecomputeStateConfig config;
    config.epsilon = {0.78, 0.78, 0.78};
    config.rotation_matrix_contactee = {
        0.000, 1.000, 0.000, 1.000, 0.000, 0.000, 0.000, 0.000, -1.000};
    config.vel_plus.fill(0.);
    config.mirror_y = true;
    return config;
}

RecomputeStateConfig get_recompute_state_config(ContactItems item)
{
    if (item == ContactItems::Robot1) return get_robot1_recompute_config();
    if (item == ContactItems::Robot2) return get_robot2_recompute_config();
    return get_table_recompute_config();
}

template <int size>
void print(std::string label, double a[size])
{
    std::cout << label << " : ";
    for (int i = 0; i < size; i++)
    {
        std::cout << a[i] << " , ";
    }
    std::cout << "\n";
}

template <int size>
void print(std::string label, const std::array<double, size>& a)
{
    std::cout << label << " : ";
    for (int i = 0; i < size; i++)
    {
        std::cout << a[i] << " , ";
    }
    std::cout << "\n";
}

static void get_rotations(const RecomputeStateConfig& config,
                          const internal::ContactStates& pre_contact,
                          std::array<double, 4>& rotation,
                          std::array<double, 4>& rotation_neg)
{
    double rotation_matrix_contactee_relative[9];
    mju_mulMatMatT(rotation_matrix_contactee_relative,
                   config.rotation_matrix_contactee.data(),
                   pre_contact.contactee_orientation.data(),
                   3,
                   3,
                   3);
    mju_mat2Quat(rotation.data(), rotation_matrix_contactee_relative);
    mju_negQuat(rotation_neg.data(), rotation.data());
}

static internal::ContactStates in_relative_frame(
    const std::array<double, 4>& rotation,
    const internal::ContactStates& pre_contact)
{
    internal::ContactStates states;

    std::array<double, 3> translated;
    for (size_t i = 0; i < 3; i++)
    {
        translated[i] =
            pre_contact.ball_position[i] - pre_contact.contactee_position[i];
    }

    mju_rotVecQuat(
        states.ball_position.data(), translated.data(), rotation.data());

    mju_rotVecQuat(states.ball_velocity.data(),
                   pre_contact.ball_velocity.data(),
                   rotation.data());

    mju_rotVecQuat(states.contactee_velocity.data(),
                   pre_contact.contactee_velocity.data(),
                   rotation.data());

    return states;
}

static internal::ContactStates in_absolute_frame(
    const std::array<double, 4>& rotation_negative,
    const internal::ContactStates& relative,
    const internal::ContactStates& pre_contact)
{
    internal::ContactStates states;

    mju_rotVecQuat(states.ball_position.data(),
                   relative.ball_position.data(),
                   rotation_negative.data());

    for (size_t i = 0; i < 3; i++)
    {
        states.ball_position[i] += pre_contact.contactee_position[i];
    }

    mju_rotVecQuat(states.ball_velocity.data(),
                   relative.ball_velocity.data(),
                   rotation_negative.data());

    return states;
}

// generic version
bool recompute_state_after_contact(const RecomputeStateConfig& config,
                                   const internal::ContactStates& pre_contact,
                                   const internal::ContactStates& current,
                                   double get_ball_position[3],
                                   double get_ball_velocity[3])
{
    // rotations
    std::array<double, 4> rotation, rotation_negative;
    get_rotations(config, pre_contact, rotation, rotation_negative);

    // pre-contact in contactee relative frame
    internal::ContactStates pre_contact_relative =
        in_relative_frame(rotation, pre_contact);

    int axis;
    if (config.mirror_y)
    {
        axis = 1;  // racket: y axis
    }
    else
    {
        axis = 2;  // table: z axis
    }

    double time_until_impact = -pre_contact_relative.ball_position[axis] /
                               (pre_contact_relative.ball_velocity[axis] -
                                pre_contact_relative.contactee_velocity[axis]);
    double delta_t_step = current.time_stamp - pre_contact.time_stamp;
    double delta_t_after_impact = delta_t_step - time_until_impact;

    // the contact is not occuring during this time step,
    // exiting
    if (delta_t_after_impact < 0)
    {
        return false;
    }

    // post-contact in relative frame
    internal::ContactStates post_contact_relative;

    // 1. velocity
    // loss of velocity after impact
    for (std::size_t i = 0; i < 3; i++)
    {
        post_contact_relative.ball_velocity[i] =
            pre_contact_relative.ball_velocity[i] * config.epsilon[i];
    }
    // change of direction
    post_contact_relative.ball_velocity[axis] =
        -pre_contact_relative.ball_velocity[axis] * config.epsilon[axis] +
        (1.0 + config.epsilon[axis]) *
            pre_contact_relative.contactee_velocity[axis];

    // 2. position
    for (size_t i = 0; i < 3; i++)
    {
        post_contact_relative.ball_position[i] =
            pre_contact_relative.ball_position[i] +
            pre_contact_relative.ball_velocity[i] * time_until_impact +
            post_contact_relative.ball_velocity[i] * delta_t_after_impact;
    }

    // final result in absolute frame
    internal::ContactStates absolute = in_absolute_frame(
        rotation_negative, post_contact_relative, pre_contact);

    // copying/returning final result
    for (size_t i = 0; i < 3; i++)
    {
        get_ball_position[i] = absolute.ball_position[i];
        get_ball_velocity[i] = absolute.ball_velocity[i] + config.vel_plus[i];
    }

    return true;
}

}  // namespace internal

}  // namespace pam_mujoco
