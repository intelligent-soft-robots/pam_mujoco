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
  MujocoItemControl();
    MujocoItemControl(MujocoItemTypes _type,
                      std::string _segment_id,
                      std::string _joint,
                      int _index_qpos,
                      int _index_qvel,
                      std::string _geometry,
                      bool _active_only,
                      std::string _configuration_path,
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
                configuration_path,
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
    char configuration_path[200];
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
    void set_racket_and_table(std::string _racket1_geometry,
			      std::string _racket2_geometry,
                              std::string _table_geometry);
    void add_control(MujocoItemControl mic);

public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(mujoco_id,
                model_path,
                burst_mode,
                accelerated_time,
                controls,
                table_geometry,
                racket1_geometry,
		racket2_geometry);
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

void set_mujoco_config(const MujocoConfig& config);
bool get_mujoco_config(const std::string& mujoco_id, MujocoConfig& get);
void wait_for_mujoco_config(const std::string& mujoco_id,
                            MujocoConfig& get,
                            bool verbose = true);

}  // namespace pam_mujoco
