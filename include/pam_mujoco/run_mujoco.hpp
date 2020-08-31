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
  
  void add_mirror_robot(std::string mujoco_id,
			std::string segment_id,
			std::string ball_obj_joint);

  template<int NB_BALLS>
  void add_mirror_balls(std::string segment_id,
			std::string ball_obj_joint);

  void add_mirror_one_ball(std::string mujoco_id,
			   std::string segment_id,
			   std::string ball_obj_joint);

  void add_contact_ball(std::string segment_id_contact,
			std::string segment_id_reset,
			std::string ball_obj_joint,
			std::string ball_geom,
			std::string contactee_geom,
			const RecomputeStateConfig& config);
  
  void add_default_contact_ball(std::string segment_id_contact,
				std::string segment_id_reset,
				std::string ball_obj_joint,
				std::string ball_geom,
				std::string contactee_geom);
  
  void add_bursting(std::string mujoco_id,std::string segment_id);
  
  void init_mujoco();
  void execute(std::string mujoco_id, std::string model_path);

  #include "run_mujoco.hxx"
  
}
