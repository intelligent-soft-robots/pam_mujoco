#pragma once

#include <unistd.h>
#include <cstring>
#include "shared_memory/shared_memory.hpp"


namespace pam_mujoco
{

  class Config
  {
  public:
    Config()
      :burst_mode{false},
       accelerated_time{false}
    {}
    Config(std::string _mujoco_id)
      :Config()
    {
      strcpy(mujoco_id,_mujoco_id.c_str());
    }
    void set_burst_mode(bool use_burst)
    {
      burst_mode=use_burst;
    }
    void set_accelerated_time(bool use_accelerated)
    {
      accelerated_time=use_accelerated;
    }
    void set_model_path(std::string path)
    {
      strcpy(model_path,path.c_str());
    }
    template <class Archive>
    void serialize(Archive& archive)
    {
      archive(mujoco_id,model_path,burst_mode,accelerated_time);
    }
    std::string to_string() const
    {
      std::stringstream ss;
      ss << "--- configuration for mujoco: " << mujoco_id << "\n"
	 << "\t burst mode: " << burst_mode << "\n"
	 << "\t model path: " << model_path << "\n"
	 << "\t accelerated time: " << accelerated_time << "\n"
      return ss.str();
    }
  public:
    char model_path[200];
    bool burst_mode;
    bool accelerated_time;
    char mujoco_id[200];
  };


  void set_mujoco_config(const Config& config)
  {
    shared_memory::serialize(std::string(config.mujoco_id),"config",config);
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
