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
    set_mujoco_started(mujoco_id,false);
    set_stopped(mujoco_id);
  }
  
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
	std::cout << mujoco_id << " : "<< value <<"\n";
      }
    catch(shared_memory::Non_existing_segment_exception)
      {
	std::cout << mujoco_id << " : error\n";
	return false;
      }
    return value;
  }
  
  void wait_for_mujoco(std::string mujoco_id)
  {
    while (! has_mujoco_started(mujoco_id) )
      {
	usleep(100);
      }
  }

}
