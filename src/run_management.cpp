#include "pam_mujoco/run_management.hpp"

namespace pam_mujoco
{

  bool BurstingMode::activated = true;
  
  bool is_bursting_mode()
  {
    return BurstingMode::activated;
  }
  
  void clear(std::string mujoco_id)
  {
    shared_memory::clear_shared_memory(mujoco_id);
  }
  
  void set_started(std::string mujoco_id)
  {
    shared_memory::set<bool>(mujoco_id,mujoco_id,true);
    // init for the request_reset and must_reset
    // functions
    shared_memory::set<bool>(mujoco_id,"reset",false);
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
    return !value;
  }

  void set_mujoco_started(std::string mujoco_id,bool value)
  {
    shared_memory::set<bool>(mujoco_id,"started",value);
  }
  
  bool has_mujoco_started(const std::string& mujoco_id)
  {
    bool value;
    bool create=false;
    try
      {
	shared_memory::get<bool>(mujoco_id,"started",value,create);
      }
    catch(shared_memory::Non_existing_segment_exception)
      {
	return false;
      }
    return value;
  }
  
  void wait_for_mujoco(std::string mujoco_id,int timeout_ms)
  {
    shared_memory::wait_for_segment(mujoco_id,timeout_ms);
  }

  void request_reset(std::string mujoco_id)
  {
    shared_memory::set<bool>(mujoco_id,"reset",true);
  }

  bool must_reset(const std::string& mujoco_id)
  {
    // note: shared memory initialized in the
    // set_started function (in this file)
    bool r;
    shared_memory::get<bool>(mujoco_id,"reset",r);
    if(r)
      {
	shared_memory::set<bool>(mujoco_id,"reset",false);
      }
    return r;
  }

}
