#include "pam_mujoco/mujoco_config.hpp"

namespace pam_mujoco
{
  MujocoItemControl::MujocoItemControl()
  {}
MujocoItemControl::MujocoItemControl(MujocoItemTypes _type,
                                     std::string _segment_id,
                                     std::string _joint,
                                     int _index_qpos,
                                     int _index_qvel,
                                     std::string _geometry,
                                     bool _active_only,
                                     std::string _configuration_path,
                                     ContactTypes _contact_type)
    : type{_type},
      index_qpos{_index_qpos},
      index_qvel{_index_qvel},
      active_only{_active_only},
      contact_type{_contact_type}
{
    strcpy(segment_id, _segment_id.c_str());
    strcpy(joint, _joint.c_str());
    strcpy(geometry, _geometry.c_str());
    strcpy(configuration_path, _configuration_path.c_str());
}

std::string MujocoItemControl::to_string() const
{
    std::stringstream ss;
    ss << "\t";
    if (type == MujocoItemTypes::ball) ss << "ball";
    if (type == MujocoItemTypes::hit_point) ss << "hit_point";
    if (type == MujocoItemTypes::goal) ss << "goal";
    if (type == MujocoItemTypes::joints) ss << "robot joints control";
    if (type == MujocoItemTypes::pressures) ss << "robot pressure control";
    ss << " |\tsegment_id: " << segment_id << " ";
    if (contact_type==ContactTypes::table)
    {
        ss << "(interrupted on contact with table) ";
    }
    if (contact_type==ContactTypes::racket1)
    {
        ss << "(interrupted on contact with racket 1) ";
    }
    if (contact_type==ContactTypes::racket2)
    {
        ss << "(interrupted on contact with racket 2) ";
    }
    if (active_only)
    {
        ss << "active control only ";
    }
    if (!std::string(configuration_path).compare(std::string("")))
    {
        ss << "configuration " << configuration_path << " ";
    }
    return ss.str();
}

MujocoConfig::MujocoConfig() : burst_mode{false}, accelerated_time{false}
{
}

MujocoConfig::MujocoConfig(std::string _mujoco_id) : MujocoConfig()
{
    strcpy(mujoco_id, _mujoco_id.c_str());
}

void MujocoConfig::set_burst_mode(bool use_burst)
{
    burst_mode = use_burst;
}

void MujocoConfig::set_accelerated_time(bool use_accelerated)
{
    accelerated_time = use_accelerated;
}

void MujocoConfig::set_model_path(std::string path)
{
    strcpy(model_path, path.c_str());
}

std::string MujocoConfig::to_string() const
{
    std::stringstream ss;
    ss << "--- configuration for mujoco: " << mujoco_id << "\n"
       << "\t burst mode: " << burst_mode << "\n"
       << "\t model path: " << model_path << "\n"
       << "\t accelerated time: " << accelerated_time << "\n";
    for (const MujocoItemControl& mic : controls)
    {
        ss << mic.to_string() << "\n";
    }
    return ss.str();
}

void MujocoConfig::set_racket_and_table(std::string _racket1_geometry,
					std::string _racket2_geometry,
                                        std::string _table_geometry)
{
  strcpy(racket1_geometry, _racket1_geometry.c_str());
  strcpy(racket2_geometry, _racket2_geometry.c_str());
  strcpy(table_geometry, _table_geometry.c_str());
}

void MujocoConfig::add_control(MujocoItemControl mic)
{
    controls.push_back(mic);
}

void set_mujoco_config(const MujocoConfig& config)
{
    shared_memory::serialize(std::string(config.mujoco_id), "config", config);
}

bool get_mujoco_config(const std::string& mujoco_id, MujocoConfig& get)
{
    try
    {
        shared_memory::deserialize(mujoco_id, "config", get);
    }
    catch (...)
    {
        return false;
    }
    return true;
}

void wait_for_mujoco_config(const std::string& mujoco_id,
                            MujocoConfig& get,
                            bool verbose)
{
    bool received = false;
    bool printed = false;
    while (true)
    {
        received = get_mujoco_config(mujoco_id, get);
        if (received)
        {
            return;
        }
        else
        {
            if (verbose && !printed)
            {
                std::cout << "waiting for configuration (" << mujoco_id
                          << ") ..." << std::endl;
                printed = true;
            }
            usleep(50000);
        }
    }
}

}  // namespace pam_mujoco
