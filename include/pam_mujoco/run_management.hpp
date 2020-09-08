#pragma once

#include <string>
#include "shared_memory/shared_memory.hpp"


namespace pam_mujoco
{

  void clear(std::string mujoco_id);
  void set_started(std::string mujoco_id);
  void set_stopped(std::string mujoco_id);
  void request_stop(std::string mujoco_id);
  bool is_stop_requested(const std::string& mujoco_id);
  void set_mujoco_started(std::string mujoco_id,bool value);
  bool has_mujoco_started(std::string mujoco_id);
  void wait_for_mujoco(std::string mujoco_id);
    
  
}
