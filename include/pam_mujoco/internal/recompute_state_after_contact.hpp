#pragma once

#include <algorithm>
#include "mujoco.h"
#include "pam_mujoco/internal/contact_states.hpp"
#include "pam_mujoco/contact_items.hpp"

namespace pam_mujoco
{
  namespace internal
  {
  
    /**
     * Configuration parameters required to compute
     * the position and velocity of the ball after contact
     * with a "contactee" (i.e. ball or racket)
     */
    class RecomputeStateConfig
    {
    public:
      RecomputeStateConfig();
      std::array<double, 3> epsilon;
      std::array<double, 3> vel_plus;
      std::array<double, 9> rotation_matrix_contactee;
      bool mirror_y;
    };

    RecomputeStateConfig get_table_recompute_config();

    RecomputeStateConfig get_robot1_recompute_config();

    RecomputeStateConfig get_robot2_recompute_config();

    RecomputeStateConfig get_recompute_state_config(ContactItems item);

    /**
     * Assuming there is a contact between the ball and the contactee
     * (i.e. racket or table), compute the position and velocity of the
     * ball resulting from this contact.
     * @param config configuration of the contact model
     * @param pre_contact state of the ball and of the contactee previous to
     *                    the contact
     * @param current current state of the ball and of the contactee
     * @param get_ball_position values of this array will be updated
     *                          with the resulting position of the ball
     * @param get_ball_velocity values of this array will be updated
     *                          with the resulting velocity of the ball
     */
    void recompute_state_after_contact(const RecomputeStateConfig& config,
				       const internal::ContactStates& pre_contact,
				       const internal::ContactStates& current,
				       double get_ball_position[3],
				       double get_ball_velocity[3]);

  }

}  // namespace pam_mujoco
