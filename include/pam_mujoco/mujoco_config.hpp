#pragma once

#include <unistd.h>
#include <cstring>
#include "shared_memory/shared_memory.hpp"


namespace pam_mujoco
{

  class Config
  {
  public:
    void set_model_path(std::string path)
    {
      strcpy(model_path,path.c_str());
    }
    template <class Archive>
    void serialize(Archive& archive)
    {
      archive(model_path);
    }
  public:
    char model_path[200];
  };


  void set_mujoco_config(const std::string& mujoco_id,
			 const Config& config)
  {
    shared_memory::serialize(mujoco_id,"config",config);
  }

  bool get_mujoco_config(const std::string& mujoco_id,
			 Config& get)
  {
    try
      {
	shared_memory::deserialize(mujoco_id,"config",get);
      }
    catch(...)
      {
	return false;
      }
    return true;
  }

  void wait_for_mujoco_config(const std::string& mujoco_id,
			      Config& get,
			      bool verbose=true)
  {
    bool received=false;
    bool printed=false;
    while(true)
      {
	received = get_mujoco_config(mujoco_id,get);
	if (received)
	  {
	    return;
	  }
	else
	  {
	    if(verbose && !printed)
	      {
		std::cout << "waiting for configuration ("
			  << mujoco_id << ") ..." << std::endl;
		printed=true;
	      }
	    usleep(50000);
	  }
      }
  }
  
}
