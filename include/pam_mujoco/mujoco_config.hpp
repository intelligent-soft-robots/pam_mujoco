#pragma once

#include <unistd.h>
#include <cstring>
#include "shared_memory/shared_memory.hpp"


namespace pam_mujoco
{

  enum MujocoItemTypes
    {
      ball,
      hitpoint,
      goal,
      joints,
      pressures
    };

  enum ContactTypes
    {
      no_contact,
      table,
      racket1,
      racket2
    };

    

  
  class MujocoItemControl
  {
  public:
    MujocoItemControl(MujocoItemTypes _type,
		      std::string _segment_id,
		      std::string _joint,
		      int _index_qpos,
		      int _index_qvel,
		      std::string _geometry,
		      bool _active_only,
		      std::string _configuration_path,
		      ContactTypes _contact_type)
      :type{_type},
       index_qpos{_index_qpos},
       index_qvel{_index_qvel},
       active_only{_active_only},
       contact_type{_contact_type}
    {
      segment_id = strcpy(segment_id,_segment_id.c_str());
      geometry = strcpy(geometry,_geometry.c_str());
      contactee_geometry = strcpy(contactee_geometry,_contactee_geometry.c_str());
      configuration_path = strcpy(configuration_path,_configuration_path.c_str());
    }
  public:
    bool until_contact() const
    {
      return !std::string::compare(std::string(contact_segment_id),
				   std::string(""));
    }
    std::string to_string() const
    {
      std::stringstream ss;
      ss << "\t";
      if(type==MujocoItemTypes::ball)
	ss << "ball";
      if(type==MujocoItemTypes::hitpoint)
	ss << "hitpoint";
      if(type==MujocoItemTypes::goal)
	ss << "goal";
      if(type==MujocoItemTypes::joints)
	ss << "robot joints control";
      if(type==MujocoItemTypes::pressure)
	ss << "robot pressure control";
      ss << " |\tsegment_id: " << segment_id << " ";
      if(until_contact())
	{
	  ss << "(interrupted on contact with "<<contact_segment_id<<") ";
	}
      if(active_only)
	{
	  ss << "active control only ";
	}
      if(!std::string::compare(std::string(configuration_path),
			       std::string("")))
	{
	  ss << "configuration "<<configuration_path<<" ";
	}
      return ss.str();
    }
  public:
    template <class Archive>
    void serialize(Archive& archive)
    {
      archive(type,segment_id,index_qpos,index_qvel,
	      geometry,
	      configuration_path,active_only);
    }
  public:
    MujocoItemTypes type;
    char segment_id[200];
    bool active;
    int index_qpos;
    int index_qvel;
    char geometry[100];
    char configuration_path[200];
    bool active_only;
    ContactTypes contact_type;
  };
  

  
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
    std::string to_string() const
    {
      std::stringstream ss;
      ss << "--- configuration for mujoco: " << mujoco_id << "\n"
	 << "\t burst mode: " << burst_mode << "\n"
	 << "\t model path: " << model_path << "\n"
	 << "\t accelerated time: " << accelerated_time << "\n";
      for(const MujocoItemControl& mic: controls)
	{
	  ss << mic.to_string() << "\n";
	}
      return ss.str();
    }
    void set_racket_and_table(std::string _racket_geometry,
			      std::string _table_geometry)
    {
      strcpy(racket_geometry,_racket_geometry);
      strcpy(table_geometry,_table_geometry);
    }
    void add_control(MujocoItemControl mic)
    {
      controls.push_back(mic);
    }
  public:
    template <class Archive>
    void serialize(Archive& archive)
    {
      archive(mujoco_id,model_path,burst_mode,accelerated_time,
	      controls,table_geometry,racket_geometry);
    }

  public:
    char model_path[200];
    bool burst_mode;
    bool accelerated_time;
    char mujoco_id[200];
    std::vector<MujocoItemControl> controls;
    char table_geometry[100];
    char racket1_geometry[100];
    char racket2_geometry[100];
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
