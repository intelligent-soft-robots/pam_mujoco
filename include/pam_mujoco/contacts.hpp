#pragma once

#include "mujoco.h"
#include <algorithm>
#include "shared_memory/shared_memory.hpp"
#include "shared_memory/serializer.hpp"
#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
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
				     const ContactStates& pre_contact,
				     const ContactStates& current,
				     double get_ball_position[3],
				     double get_ball_velocity[3]);

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
		  int index_qvel_ball,
		  int index_geom_ball,
		  int index_geom_contactee,
		  ContactStates& get_states);


  /**
   * Encapsulates the information about a contact
   * (or the abscence of contact). 
   * position and time_stamp
   * have meaning only if contact_occured is true.
   */
  class ContactInformation
  {
  public:
    ContactInformation();
    void register_distance(double d);
    void register_contact(std::array<double,3> position,
			  double time_stamp);
    template <class Archive>
    void serialize(Archive &archive)
    {
      archive(position,
	      contact_occured,
	      time_stamp,
	      minimal_distance);
    }
  public:
    std::array<double,3> position;
    bool contact_occured;
    double time_stamp;
    double minimal_distance;
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
    ContactBall(std::string segment_id_contact_info,
		std::string segment_id_reset,
		RecomputeStateConfig config,
		std::string ball_obj_joint,
		std::string ball_geom,
		std::string contactee_geom);
  public:
    // ControllerBase function
    void apply(const mjModel* m,
	       mjData* d);
  private:
    void manage_contact(const mjModel* m, mjData* d);
    void init(const mjModel* m);
    bool is_muted(bool contact_detected);
    bool is_in_contact(const mjModel* m, mjData* d);
    void reset();
  private:
    std::string segment_id_contact_info_;
    std::string segment_id_reset_;
    RecomputeStateConfig config_;
    ContactInformation contact_information_;
    ContactStates previous_;
    int index_q_ball_;
    int index_qvel_ball_;
    int index_geom_ball_;
    int index_geom_contactee_; // contactee : racket or table
    std::string ball_obj_joint_;
    std::string ball_geom_;
    std::string contactee_geom_;
    bool muted_;
    int muted_count_;
  };




  
}
