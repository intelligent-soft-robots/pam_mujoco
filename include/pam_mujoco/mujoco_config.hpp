#pragma once

#include <unistd.h>
#include <cstring>
#include "shared_memory/shared_memory.hpp"

namespace pam_mujoco
{

enum MujocoItemTypes
{
    ball,
    hit_point,
    goal
};

enum ContactTypes
{
    no_contact,
    table,
    racket1,
    racket2
};

class MujocoRobotJointControl
{
public:
  MujocoRobotJointControl();
  MujocoRobotJointControl(std::string _segment_id,
			  std::string _joint,
			  bool _active_only);
  std::string to_string() const;
public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(segment_id,
		joint,
		active_only);
    }
public:
  char segment_id[200];
  char joint[200];
  bool active_only;
};

  class MujocoRobotPressureControl 
  {
  public:
    MujocoRobotPressureControl();
    MujocoRobotPressureControl(std::string _segment_id,
			       std::string _joint,
			       bool _active_only,
			       std::string _json_controller_path,
			       std::string _json_ago_hill_path,
			       std::string _json_antago_hill_path);
    std::string to_string() const;
  public:
    template <class Archive>
    void serialize(Archive& archive)
    {
      archive(segment_id,
	      joint,
	      json_controller_path,
	      json_ago_hill_path,
	      json_antago_hill_path,
	      active_only);
    }
  public:
    char segment_id[200];
    char joint[200];
    char json_controller_path[200];
    char json_ago_hill_path[200];
    char json_antago_hill_path[200];
    bool active_only;
  };

    
    
class MujocoItemControl
{
public:
  MujocoItemControl();
    MujocoItemControl(MujocoItemTypes _type,
                      std::string _segment_id,
                      std::string _joint,
                      int _index_qpos,
                      int _index_qvel,
                      std::string _geometry,
                      bool _active_only,
                      ContactTypes _contact_type);

public:
    std::string to_string() const;

public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(type,
                segment_id,
		joint,
                index_qpos,
                index_qvel,
                geometry,
                active_only,
		contact_type);
    }

public:
    MujocoItemTypes type;
    char segment_id[200];
    char joint[200];
    bool active;
    int index_qpos;
    int index_qvel;
    char geometry[100];
    bool active_only;
    ContactTypes contact_type;
};

class MujocoConfig
{
public:
    MujocoConfig();
    MujocoConfig(std::string _mujoco_id);
    void set_burst_mode(bool use_burst);
    void set_accelerated_time(bool use_accelerated);
    void set_model_path(std::string path);
    std::string to_string() const;
  void set_racket_robot1(std::string _racket1_geometry);
  void set_racket_robot2(std::string _racket2_geometry);
  void set_table(std::string _table_geometry);
  void add_control(MujocoItemControl mic);
  void add_control(MujocoRobotJointControl mrc);
  void add_control(MujocoRobotPressureControl mpc);

public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(mujoco_id,
                model_path,
                burst_mode,
                accelerated_time,
                item_controls,
		joint_controls,
		pressure_controls,
                table_geometry,
                racket1_geometry,
		racket2_geometry);
    }

public:
    char model_path[200];
    bool burst_mode;
    bool accelerated_time;
    char mujoco_id[200];
    std::vector<MujocoItemControl> item_controls;
    std::vector<MujocoRobotJointControl> joint_controls;
    std::vector<MujocoRobotPressureControl> pressure_controls;
    char table_geometry[100];
    char racket1_geometry[100];
    char racket2_geometry[100];
};

void set_mujoco_config(const MujocoConfig& config);
bool get_mujoco_config(const std::string& mujoco_id, MujocoConfig& get);
void wait_for_mujoco_config(const std::string& mujoco_id,
                            MujocoConfig& get,
                            bool verbose = true);

  // if I call this function "wait_for_mujoco" instead of "_wait_for_mujoco",
  // then pybind11 (in ../srcpy/wrappers) considers it as an override of another
  // function (and fails to compile). I have no idea why, I could not find another "wait_for_mujoco"
  // function in this package.
  void _wait_for_mujoco(const std::string& mujoco_id);
  
}  // namespace pam_mujoco
