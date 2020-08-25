#pragma once

#include <tuple>
#include <set>
#include <string>
#include "shared_memory/shared_memory.hpp"
#include "real_time_tools/thread.hpp"
#include "pam_mujoco/mujoco_base.hpp"
#include "pam_mujoco/mirror_robot.hpp"
#include "pam_mujoco/mirror_balls.hpp"
#include "pam_mujoco/burst_controller.hpp"
#include "pam_mujoco/run_management.hpp"


namespace pam_mujoco
{

  static constexpr int NB_DOFS = 4;
  static constexpr long int QUEUE_SIZE = 500000;
  
  static const std::string SEGMENT_ID_PREFIX("pam_mujoco_");

  enum ControllerTypes
    {
      MIRROR_ROBOT,
      MIRROR_ONE_BALL
    };
  
  static const std::string MIRROR_ROBOT_SUFFIX("mirror_robot");
  static const std::string MIRROR_ONE_BALL_SUFFIX("mirror_one_ball");

  
  bool run_g = true;
  std::string error_message_g("no error");
  
  void exit(const char* text);
  
  std::string get_mirror_robot_segment_id(std::string mujoco_id);
  void add_mirror_robot(std::string segment_id);

  std::string get_mirror_one_ball_segment_id(std::string mujoco_id);
  template<int NB_BALLS>
  void add_mirror_balls(std::string segment_id);

  void construct_controllers(std::string mujoco_id,
			     std::set<ControllerTypes> controller_types,
			     std::string burster_segment_id,
			     const mjModel* m,
			     const mjData* d_init);
  
  void execute(std::string mujoco_id, std::string model_path,
	       std::set<ControllerTypes> controller_types,
	       std::string burster_segment_id=std::string(""));

  #include "run_mujoco.hxx"
  
}
