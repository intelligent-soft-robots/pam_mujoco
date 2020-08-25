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
  static const std::string MIRROR_ROBOT_SUFFIX("mirror_robot");
  static const std::string MIRROR_ONE_BALL_SUFFIX("mirror_one_ball");

  void init_mujoco();
  std::string add_mirror_robot(std::string mujoco_id);
  std::string add_mirror_one_ball(std::string mujoco_id);
  std::string get_mirror_robot_segment_id(std::string mujoco_id);
  std::string get_mirror_one_ball_segment_id(std::string mujoco_id);
  void set_bursting(std::string segment_id);
  void execute(std::string mujoco_id, std::string model_path);

  #include "run_mujoco.hxx"
  
}
