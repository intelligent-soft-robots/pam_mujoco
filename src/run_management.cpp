#include "pam_mujoco/run_management.hpp"

namespace pam_mujoco
{
  
  void set_started(std::string mujoco_id)
  {
    shared_memory::set<bool>(mujoco_id,mujoco_id,true);
  }

  void set_stopped(std::string mujoco_id)
  {
    shared_memory::set<bool>(mujoco_id,mujoco_id,false);
  }
  
  void request_stop(std::string mujoco_id)
  {
    shared_memory::set<bool>(mujoco_id,mujoco_id,false);
  }

  bool is_stop_requested(const std::string& mujoco_id)
  {
    bool value;
    shared_memory::get<bool>(mujoco_id,mujoco_id,value);
    return value;
  }

}
