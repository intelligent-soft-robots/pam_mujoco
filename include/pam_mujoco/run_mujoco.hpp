#pragma once

#include <tuple>
#include <set>
#include <string>
#include "shared_memory/shared_memory.hpp"
#include "real_time_tools/thread.hpp"
#include "pam_mujoco/mujoco_base.hpp"
#include "pam_mujoco/mirror_robot.hpp"
#include "pam_mujoco/mirror_balls.hpp"
#include "pam_mujoco/contacts.hpp"
#include "pam_mujoco/burst_controller.hpp"
#include "pam_mujoco/run_management.hpp"


namespace pam_mujoco
{

  static constexpr int NB_DOFS = 4;
  static constexpr long int QUEUE_SIZE = 500000;
  
  bool run_g = true;
  std::string error_message_g("no error");
  
  void exit(const char* text);
  
  /**
   * Add a mirror robot controller, which will compute desired
   * robot state using o80 running on segment_id. This controller
   * will update at each mujoco iteration the mjModel 
   * to get the mirrored robot to match the desired state as set
   * by correspondig o80 frontend. See demos/mirror_robot.py (in this package)
   * to see such frontend in action
   * @param mujoco_id : id of the mujoco instance
   * @param segment_id: o80 segment id
   * @param robot_joint_base: as set in the xml model
   */
  void add_mirror_robot(std::string mujoco_id,
			std::string segment_id,
			std::string robot_joint_base);

  template<int NB_BALLS>
  void add_mirror_balls(std::string segment_id,
			std::string ball_obj_joint,
			const std::map<int,std::string>& ball_index_segment_id);

  /**
   * Add a mirror ball controller, which will compute desired
   * ball state using o80 running on segment_id. This controller
   * will update at each mujoco iteration the mjModel 
   * to get the mirrored ball to match the desired state as set
   * by correspondig o80 frontend. See demos/play_trajectory.py (in this package)
   * to see such frontend in action
   * @param mujoco_id : id of the mujoco instance
   * @param segment_id: o80 segment id
   * @param ball_obj_joint: as set in the xml model
   */
  void add_mirror_one_ball(std::string mujoco_id,
			   std::string segment_id,
			   std::string ball_obj_joint);

  /**
   * Similar to add_mirror_one_ball with the exception that upon
   * contact of the ball, as set in the shared memory in the segment
   * "contact_segment_id", the controller will inactivate (i.e. will 
   * no longer update mjData, i.e. the ball will move according to 
   * either mujoco physics or other controller). 
   * Warning: this assumes a contact controller running over 
   * contact_segment_id (see add_contact_ball).
   * @param mujoco_id : id of the mujoco instance
   * @param segment_id: o80 segment id
   * @param ball_obj_joint: as set in the xml model
   */
  void add_mirror_until_contact_one_ball(std::string mujoco_id,
					 std::string segment_id,
					 std::string ball_obj_joint,
					 std::string contact_segment_id);

  void add_contact_ball(std::string segment_id_contact,
			std::string segment_id_reset,
			std::string ball_obj_joint,
			std::string ball_geom,
			std::string contactee_geom,
			const RecomputeStateConfig& config);

  /**
   * Add a contact controller which:
   *   - detect contact between the ball and the contactee
   *   - upon contact, update mjData to get the ball to move
   *     according to a customized contact model.
   * Also shares at each iteration contact information by 
   * serializing an instance of ContactInformation (see Contacts.hpp)
   * into the shared memory segment segment_id_contact.
   * The instance of contact information is reset (i.e. contact_occured
   * set to false) if the boolean hosted by the shared memory upon
   * segment "segment_id_reset" is set to True.
   */
  void add_default_contact_ball(std::string segment_id_contact,
				std::string segment_id_reset,
				std::string ball_obj_joint,
				std::string ball_geom,
				std::string contactee_geom);
  
  /**
   * Add a bursting controller, i.e. a controller which encapsulates
   * an instance of o80::Burster running over segment_id. 
   * This will have the mujoco instance "blocked" until the
   * "burst" method of a corresponding o80 frontend is called
   */
  void add_bursting(std::string mujoco_id,std::string segment_id);
  
  void init_mujoco();
  void execute(std::string mujoco_id, std::string model_path);

  #include "run_mujoco.hxx"
  
}
