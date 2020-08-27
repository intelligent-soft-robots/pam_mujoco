#pragma once

#include <algorithm>
#include "shared_memory/serializer.hpp"
#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{

  /**
   * Encapsulate the state (i.e. position and velocity)
   * of a ball and another object (i.e. racket and table)
   */
  class ContactsStates
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
    ContactsStates();
  public:
    // contactee: likely to be a racket or a table
    std::array<double,9> contactee_orientation;
    std::array<double,3> contactee_position;
    std::array<double,3> contactee_velocity;
    std::array<double,3> ball_position;
    std::array<double,3> ball_velocity;
    double time_stamp;
  };


  /**
   * Configuration parameters required to compute
   * the position and velocity of the ball after contact
   * with a "contactee" (i.e. ball or racket)
   */
  class RecomputeStateConfig
  {
  public:
    RecomputeStateConfig(bool robot1);
    double epsilon_r;
    double epsilon_t_x;
    double epsilon_t_z;
    double y_vel_plus;
    double z_vel_plus;
    std::array<double,9> rot_matrix_contactee_zero_pos;
  };


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
				     const ContactsStates& pre_contact,
				     const ContactsStates& current,
				     double[3] get_ball_position,
				     double[3] get_ball_velocity);

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
		  int index_q_ball,
		  int index_geom_ball,
		  int index_geom_contactee,
		  ContactsStates& get_states);

  /**
   * class for managing the contact between the ball
   * and the contactee (i.e. racket or table), i.e. 
   * detecting contacts and recomputing resulting ball
   * position and velocity
   */
  class ContactBall
  {

  public:
    ContactBall(RecomputeStateConfig config,
		int index_q_ball,
		int index_geom_ball,
		int index_geom_contactee);
    /**
     * detect contacts, and if contact is detected,
     * update the position and velocity of the ball
     * encapsulated in d.
     */
    void manage_contact(const mjModel* m, mjData* d);
  private:
    bool is_muted(bool contact_detected);
    bool is_in_contact(const mjModel* m, mjData* d);
  private:
    // contactee : racket or table
    RecomputeStateConfig config_;
    ContactsStates previous_;
    double min_distance_ball_contactee_;
    int index_q_ball_;
    int index_geom_ball;
    int index_geom_contactee_;
    bool contact_detected_;
    bool muted_;
    int muted_count_;
  };


}
