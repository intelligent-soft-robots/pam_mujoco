#pragma once

#include <set>
#include <string>
#include "shared_memory/shared_memory.hpp"
#include "real_time_tools/thread.hpp"
#include "pam_mujoco/mujoco_base.hpp"
#include "pam_mujoco/mirror_external_robot.hpp"
#include "pam_mujoco/run_management.hpp"


namespace pam_mujoco
{

  static constexpr int NB_DOFS = 4;
  static constexpr long int QUEUE_SIZE = 500000;
  
  static const std::string SEGMENT_ID_PREFIX("pam_mujoco_");

  static constexpr int MIRROR_EXTERNAL_ROBOT = 1;
  static const std::string MIRROR_EXTERNAL_ROBOT_SUFFIX("mirror_robot");
  
  static constexpr int MIRROR_EXTERNAL_BALL = 2;
  static const std::string MIRROR_EXTERNAL_BALL_SUFFIX("mirror_ball");

  bool run_g = true;
  std::string error_message_g("no error");
  
  void exit(const char* text);
  std::string get_mirror_external_robot_segment_id(std::string mujoco_id);
  void add_mirror_external_robot(std::string segment_id,
				 const mjModel* m,
				 const mjData* d_init);
  void construct_controllers(std::set<int> controller_ids,
			     const mjModel* m,
			     const mjData* d_init);
  void execute(std::string mujoco_id, std::string model_path,
	       std::set<int> controller_ids);

}
